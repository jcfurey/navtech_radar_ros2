// SPDX-License-Identifier: Apache-2.0
#include "recording.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <zlib.h>

namespace recording {
namespace {
constexpr size_t max_packet = 4 * 1024 * 1024;
constexpr uint8_t signature[] = {0,  1,  3,  3,  7,   7,   15,  15,
                                 31, 31, 63, 63, 127, 127, 254, 254};
void read_exact(std::istream &in, uint8_t *dst, size_t size) {
  require(bool(in.read(reinterpret_cast<char *>(dst), size)),
          "truncated capture record");
}
float float_be(const uint8_t *p) {
  uint32_t bits = big(p, 4);
  float value;
  std::memcpy(&value, &bits, 4);
  return value;
}
bool next_video(std::ifstream &in, uint16_t &bearing, uint64_t &ns) {
  while (in.peek() != EOF) {
    std::array<uint8_t, 16> h;
    read_exact(in, h.data(), h.size());
    auto type = big(h.data() + 2, 2), size = big(h.data() + 4, 4);
    require(h[0] == 'C' && h[1] == 'B' && type >> 8 == 1 && size >= 16 &&
                size <= 16 * 1024 * 1024,
            "invalid CPR header");
    if (type == 0x102) {
      require(size >= 18 && big(h.data() + 12, 4) < 1000000,
              "invalid CPR video");
      std::array<uint8_t, 2> angle;
      read_exact(in, angle.data(), 2);
      bearing = big(angle.data(), 2);
      ns = uint64_t(big(h.data() + 8, 4)) * 1000000000 +
           uint64_t(big(h.data() + 12, 4)) * 1000;
      in.ignore(size - 18);
      require(in.gcount() == size - 18, "truncated CPR video");
      return true;
    }
    in.ignore(size - 16);
    require(in.gcount() == size - 16, "truncated CPR record");
  }
  return false;
}
Bytes inflate_record(const Bytes &compressed) {
  // The inner record length is available after inflating just its 13-byte
  // header.
  z_stream z{};
  require(inflateInit2(&z, -15) == Z_OK, "cannot initialize deflate decoder");
  struct End {
    z_stream &z;
    ~End() { inflateEnd(&z); }
  } end{z};
  z.next_in = const_cast<uint8_t *>(compressed.data());
  z.avail_in = compressed.size();
  Bytes inner(13);
  z.next_out = inner.data();
  z.avail_out = 13;
  auto status = inflate(&z, Z_NO_FLUSH);
  require(z.total_out == 13 && (status == Z_OK || status == Z_STREAM_END),
          "invalid compressed COLRAW header");
  auto size = little(inner.data(), 4);
  require(size <= max_packet, "oversize compressed COLRAW packet");
  inner.resize(13 + size + 1);
  z.next_out = inner.data() + 13;
  z.avail_out = size + 1;
  if (status != Z_STREAM_END)
    status = inflate(&z, Z_FINISH);
  require(status == Z_STREAM_END && !z.avail_in && z.total_out == 13 + size,
          "invalid compressed COLRAW body");
  inner.pop_back();
  return inner;
}
} // namespace
void read_colraw(const fs::path &path, bool paired, const Emit &emit) {
  auto source = path;
  std::ifstream video;
  if (paired) {
    video.open(path, std::ios::binary);
    require(bool(video), "cannot open CPR");
    source.replace_extension(".colraw");
  }
  std::ifstream in(source, std::ios::binary);
  require(bool(in), "cannot open COLRAW " + source.string());
  navtech_radar_msgs::msg::RadarConfiguration cfg;
  Rotation turn;
  uint32_t mhz = 0;
  auto flush = [&] {
    if (turn.encoder_values.empty())
      return;
    turn.image.height = turn.encoder_values.size();
    turn.image.width = cfg.range_bins;
    turn.image.step =
        cfg.range_bins * (turn.configuration.sample_bit_depth / 8);
    turn.image.encoding =
        turn.configuration.sample_bit_depth == 8 ? "mono8" : "mono16";
    auto t = turn.sensor_ns.front() ? turn.sensor_ns.front()
                                    : turn.receipt_ns.front();
    emit(t, "recording/rotation", std::move(turn));
    turn = Rotation{};
  };
  Bytes bytes;
  while (in.peek() != EOF) {
    check_running();
    std::array<uint8_t, 13> header;
    read_exact(in, header.data(), header.size());
    auto size = little(header.data(), 4);
    require(size <= max_packet, "oversize COLRAW packet");
    bytes.resize(size);
    read_exact(in, bytes.data(), size);
    const uint8_t *h = header.data(), *packet = bytes.data();
    if (h[4] == 255) {
      bytes = inflate_record(bytes);
      h = bytes.data();
      size = little(h, 4);
      packet = h + 13;
    }
    auto type = h[4];
    if (type != 10 && type != 30 && type != 31)
      continue;
    require(
        size >= 22 &&
            std::equal(std::begin(signature), std::end(signature), packet) &&
            packet[17] == type && big(packet + 18, 4) == size - 22,
        "invalid Colossus packet");
    auto data = packet + 22;
    size -= 22;
    if (type == 10) {
      flush();
      require(size >= 20, "short Colossus configuration");
      cfg.azimuth_samples = big(data, 2);
      cfg.bin_size = big(data + 2, 2);
      cfg.range_bins = big(data + 4, 2);
      cfg.encoder_size = big(data + 6, 2);
      require(cfg.azimuth_samples && cfg.azimuth_samples <= 2048 &&
                  cfg.azimuth_samples <= cfg.encoder_size && cfg.range_bins &&
                  cfg.range_bins <= 16384 && cfg.bin_size,
              "invalid Colossus geometry");
      mhz = big(data + 8, 2);
      cfg.expected_rotation_rate = std::nearbyint(mhz / 1000.);
      cfg.packet_rate = big(data + 10, 2);
      cfg.range_resolution = cfg.bin_size / 10000.;
      cfg.range_gain = float_be(data + 12);
      cfg.range_offset = float_be(data + 16);
      continue;
    }
    require(cfg.encoder_size && size >= 14,
            "missing configuration or FFT header");
    auto offset = big(data, 2), counter = big(data + 2, 2),
         enc = big(data + 4, 2);
    unsigned depth = type == 30 ? 8 : 16;
    require(offset >= 14 && offset <= size &&
                size - offset == cfg.range_bins * (depth / 8) &&
                enc < cfg.encoder_size,
            "invalid COLRAW FFT layout");
    uint64_t sensor = little(data + 6, 4) * 1000000000 + little(data + 10, 4);
    uint64_t ticks = little(h + 5, 4) << 32 | little(h + 9, 4);
    require(ticks <= UINT64_MAX / 100, "recorder time overflow");
    uint64_t host = ticks * 100;
    if (paired) {
      uint16_t angle;
      require(next_video(video, angle, host), "missing CPR video record");
      require(std::abs(std::remainder(
                  double(enc) / cfg.encoder_size - angle / 65536., 1.)) <=
                  1. / cfg.encoder_size,
              "CPR bearing mismatch");
      sensor = 0;
    } else
      require(little(data + 10, 4) < 1000000000, "invalid FFT nanoseconds");
    if (!turn.encoder_values.empty()) {
      int prev = turn.encoder_values.back(), delta = int(enc) - prev;
      auto steps = uint16_t(counter - turn.sweep_counters.back());
      auto crossed = cfg.encoder_size + delta;
      uint64_t then = turn.sensor_ns.back(), now = sensor;
      if (!then || !now) {
        then = turn.receipt_ns.back();
        now = host;
      }
      double elapsed_turns = then && now ? (now >= then ? double(now - then)
                                                        : -double(then - now)) *
                                               1e-12 * mhz
                                         : 0.;
      bool hidden =
          steps && (delta < 0 || steps >= cfg.azimuth_samples) &&
          2 * std::abs(int64_t(crossed) * cfg.azimuth_samples -
                       int64_t(steps) * cfg.encoder_size) <
              cfg.encoder_size &&
          (!then || !now || !mhz ||
           std::abs(elapsed_turns - double(crossed) / cfg.encoder_size) < 0.5);
      if (delta * 2 < -int(cfg.encoder_size) ||
          depth != turn.configuration.sample_bit_depth || elapsed_turns >= 1 ||
          hidden)
        flush();
    }
    if (turn.encoder_values.empty()) {
      turn.configuration = cfg;
      turn.configuration.sample_bit_depth = depth;
    }
    require(turn.encoder_values.size() < 2048,
            "COLRAW rotation exceeds row limit");
    turn.encoder_values.push_back(enc);
    turn.sweep_counters.push_back(counter);
    turn.sensor_ns.push_back(sensor);
    turn.receipt_ns.push_back(host);
    turn.image.data.insert(turn.image.data.end(), data + offset, data + size);
  }
  if (paired) {
    uint16_t angle;
    uint64_t time;
    require(!next_video(video, angle, time), "extra CPR video records");
  }
  flush();
}
} // namespace recording
