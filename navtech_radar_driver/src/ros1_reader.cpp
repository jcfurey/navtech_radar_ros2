// SPDX-License-Identifier: Apache-2.0
// Indexed ROS bag v2 reader for the four sensor products in Navtech's road
// export.
#include "dataset_files.hpp"
#include <algorithm>
#include <bzlib.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <lz4frame.h>
#include <map>
#include <spawn.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char **environ;

namespace recording {
namespace {
constexpr size_t max_chunk = 512UL << 20;
class Cursor {
public:
  Cursor(const uint8_t *p, size_t n) : p_(p), left_(n) {}
  size_t left() const { return left_; }
  const uint8_t *take(size_t n) {
    require(n <= left_, "truncated ROS 1 record");
    auto p = p_;
    p_ += n;
    left_ -= n;
    return p;
  }
  uint32_t u32() { return little(take(4), 4); }
  uint8_t byte() { return *take(1); }
  float f32() {
    uint32_t bits = u32();
    float f;
    std::memcpy(&f, &bits, 4);
    return f;
  }
  std::string string() {
    auto n = u32();
    return std::string(reinterpret_cast<const char *>(take(n)), n);
  }
  Bytes bytes() {
    auto n = u32();
    auto p = take(n);
    return Bytes(p, p + n);
  }
  std::vector<float> floats() {
    auto n = u32();
    require(n <= left() / 4, "truncated LaserScan array");
    std::vector<float> out(n);
    for (auto &f : out)
      f = f32();
    return out;
  }

private:
  const uint8_t *p_;
  size_t left_;
};
using Fields = std::map<std::string, std::string>;
Fields fields(const uint8_t *p, size_t n) {
  Fields out;
  Cursor c(p, n);
  while (c.left()) {
    auto kv = c.string();
    auto sep = kv.find('=');
    require(sep != kv.npos, "invalid bag field");
    out.emplace(kv.substr(0, sep), kv.substr(sep + 1));
  }
  return out;
}
uint64_t integer(const Fields &h, const char *key, unsigned bytes) {
  auto &v = h.at(key);
  require(v.size() == bytes, "invalid ROS bag integer");
  return little(reinterpret_cast<const uint8_t *>(v.data()), bytes);
}
uint32_t stream_u32(std::ifstream &f) {
  uint8_t p[4];
  require(bool(f.read(reinterpret_cast<char *>(p), 4)), "truncated bag length");
  return little(p, 4);
}
Bytes read(std::ifstream &f, size_t n, size_t limit) {
  require(n <= limit, "oversize ROS bag record");
  Bytes out(n);
  require(bool(f.read(reinterpret_cast<char *>(out.data()), n)),
          "truncated bag");
  return out;
}
std_msgs::msg::Header header(Cursor &c) {
  c.u32();
  auto sec = c.u32(), nsec = c.u32();
  require(nsec < 1000000000, "invalid sensor timestamp");
  std_msgs::msg::Header h;
  h.stamp = stamp(uint64_t(sec) * 1000000000 + nsec);
  h.frame_id = c.string();
  return h;
}
struct Connection {
  const char *topic, *type, *md5;
};
const std::map<std::string, Connection> supported = {
    {"/Navtech/Polar",
     {"fft/display", "sensor_msgs/Image", "060021388200f6f0f447d0fcd9c64743"}},
    {"/Navtech/Cartesian1",
     {"fft/cartesian", "sensor_msgs/Image",
      "060021388200f6f0f447d0fcd9c64743"}},
    {"/Navtech/scan3",
     {"scan", "sensor_msgs/LaserScan", "90c7ef2dc6895d81024acba2ac42f369"}},
    {"/Navtech/FilteredPointcloud",
     {"point_cloud", "sensor_msgs/PointCloud2",
      "1158d486dd51d683ce2f1be655c3c181"}}};
Product decode(const Connection &conn, Cursor &c) {
  auto h = header(c);
  if (std::string_view(conn.type) == "sensor_msgs/Image") {
    Image m;
    m.header = std::move(h);
    m.height = c.u32();
    m.width = c.u32();
    m.encoding = c.string();
    m.is_bigendian = c.byte();
    m.step = c.u32();
    m.data = c.bytes();
    return m;
  }
  if (std::string_view(conn.type) == "sensor_msgs/LaserScan") {
    Scan m;
    m.header = std::move(h);
    m.angle_min = c.f32();
    m.angle_max = c.f32();
    m.angle_increment = c.f32();
    m.time_increment = c.f32();
    m.scan_time = c.f32();
    m.range_min = c.f32();
    m.range_max = c.f32();
    m.ranges = c.floats();
    m.intensities = c.floats();
    return m;
  }
  Cloud m;
  m.header = std::move(h);
  m.height = c.u32();
  m.width = c.u32();
  auto count = c.u32();
  require(count <= c.left() / 13, "invalid PointField count");
  for (uint32_t i = 0; i < count; ++i) {
    auto &f = m.fields.emplace_back();
    f.name = c.string();
    f.offset = c.u32();
    f.datatype = c.byte();
    f.count = c.u32();
  }
  m.is_bigendian = c.byte();
  m.point_step = c.u32();
  m.row_step = c.u32();
  m.data = c.bytes();
  m.is_dense = c.byte();
  return m;
}

struct Chunk {
  std::streamoff pos;
  uint32_t size, raw;
  std::string compression;
};
struct Index {
  uint64_t ns;
  uint32_t conn, chunk, offset;
};
Bytes load_chunk(std::ifstream &f, const Chunk &c) {
  f.seekg(c.pos);
  auto src = read(f, c.size, max_chunk);
  if (c.compression == "none") {
    require(src.size() == c.raw, "chunk size mismatch");
    return src;
  }
  require(c.raw <= max_chunk, "oversize expanded chunk");
  Bytes out(c.raw);
  if (c.compression == "bz2") {
    unsigned int n = c.raw;
    require(BZ2_bzBuffToBuffDecompress(reinterpret_cast<char *>(out.data()), &n,
                                       reinterpret_cast<char *>(src.data()),
                                       src.size(), 0, 0) == BZ_OK &&
                n == c.raw,
            "invalid bz2 bag chunk");
  } else if (c.compression == "lz4") {
    LZ4F_dctx *ctx = nullptr;
    require(!LZ4F_isError(LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION)),
            "cannot initialize LZ4");
    std::unique_ptr<LZ4F_dctx, decltype(&LZ4F_freeDecompressionContext)>
        context(ctx, LZ4F_freeDecompressionContext);
    size_t i = 0, j = 0, hint;
    do {
      size_t input = src.size() - i, output = out.size() - j;
      hint = LZ4F_decompress(ctx, out.data() + j, &output, src.data() + i,
                             &input, nullptr);
      require(!LZ4F_isError(hint) && (input || output || !hint),
              "invalid LZ4 bag chunk");
      i += input;
      j += output;
    } while (hint);
    require(i == src.size() && j == out.size(), "invalid LZ4 chunk size");
  } else
    throw std::runtime_error("unknown bag compression " + c.compression);
  return out;
}
void bag(const fs::path &path, const Emit &emit) {
  std::ifstream f(path, std::ios::binary);
  constexpr std::string_view version = "#ROSBAG V2.0\n";
  char magic[version.size()];
  require(f.read(magic, sizeof(magic)) &&
              std::string_view(magic, sizeof(magic)) == version,
          "expected ROS bag v2");
  auto length = std::streamoff(fs::file_size(path));
  std::vector<Chunk> chunks;
  std::vector<Index> index;
  std::map<uint32_t, Connection> conns;
  while (f.tellg() < length) {
    check_running();
    auto raw = read(f, stream_u32(f), 1 << 20);
    auto h = fields(raw.data(), raw.size());
    auto n = stream_u32(f);
    std::streamoff pos = f.tellg();
    require(length - pos >= n, "truncated ROS bag record");
    switch (integer(h, "op", 1)) {
    case 5:
      chunks.push_back(
          {pos, n, uint32_t(integer(h, "size", 4)), h.at("compression")});
      break;
    case 4: {
      auto id = uint32_t(integer(h, "conn", 4));
      auto count = integer(h, "count", 4);
      require(!chunks.empty() && integer(h, "ver", 4) == 1 && count * 12 == n,
              "invalid chunk index");
      auto data = read(f, n, max_chunk);
      Cursor c(data.data(), data.size());
      for (size_t i = 0; i < count; ++i) {
        auto sec = c.u32(), nano = c.u32();
        require(nano < 1000000000, "invalid bag timestamp");
        index.push_back({uint64_t(sec) * 1000000000 + nano, id,
                         uint32_t(chunks.size() - 1), c.u32()});
      }
      break;
    }
    case 7: {
      auto found = supported.find(h.at("topic"));
      if (found != supported.end()) {
        auto data = read(f, n, 4 << 20);
        auto info = fields(data.data(), data.size());
        require(info.at("type") == found->second.type &&
                    info.at("md5sum") == found->second.md5,
                "unexpected ROS 1 message schema");
        conns.emplace(integer(h, "conn", 4), found->second);
      }
      break;
    }
    }
    f.seekg(pos + n);
    require(bool(f), "invalid bag offset");
  }
  require(!conns.empty(), "no Navtech sensor products in bag");
  index.erase(
      std::remove_if(index.begin(), index.end(),
                     [&](const Index &i) { return !conns.count(i.conn); }),
      index.end());
  std::stable_sort(index.begin(), index.end(),
                   [](const auto &a, const auto &b) {
                     return std::tie(a.ns, a.chunk, a.offset) <
                            std::tie(b.ns, b.chunk, b.offset);
                   });
  uint32_t loaded = UINT32_MAX;
  Bytes data;
  for (auto &i : index) {
    check_running();
    if (loaded != i.chunk) {
      data = load_chunk(f, chunks[i.chunk]);
      loaded = i.chunk;
    }
    require(i.offset < data.size(), "index outside ROS bag chunk");
    Cursor record(data.data() + i.offset, data.size() - i.offset);
    auto len = record.u32();
    auto h = fields(record.take(len), len);
    require(integer(h, "op", 1) == 2 && integer(h, "conn", 4) == i.conn,
            "bag index does not match message");
    auto size = record.u32();
    Cursor body(record.take(size), size);
    auto &conn = conns.at(i.conn);
    auto product = decode(conn, body);
    require(!body.left(), "trailing ROS 1 message bytes");
    emit(i.ns, conn.topic, std::move(product));
  }
}
} // namespace
void read_ros1(const fs::path &p, const Emit &emit) {
  if (extension(p) != ".zip") {
    bag(p, emit);
    return;
  }
  std::string member;
  for (auto &n : Zip(p).names())
    if (extension(n) == ".bag") {
      require(member.empty(), "multiple bags in ZIP");
      member = n;
    }
  require(!member.empty(), "no bag in ZIP");
  // Info-ZIP handles the manufacturer's Deflate64 archive. Pass the member as
  // an argument and stream stdout into a private file for indexed bag reads.
  std::string pattern;
  for (char c : member) {
    if (c == '[' || c == '?' || c == '*' || c == '\\')
      pattern += '\\';
    pattern += c;
  }
  require(pattern.front() != '-', "unsupported ZIP member name");
  TemporaryDirectory tmp;
  auto path = tmp.path / "road.bag";
  auto archive = fs::absolute(p).string();
  posix_spawn_file_actions_t actions;
  require(!posix_spawn_file_actions_init(&actions), "cannot initialize unzip");
  struct Cleanup {
    posix_spawn_file_actions_t &actions;
    ~Cleanup() { posix_spawn_file_actions_destroy(&actions); }
  } cleanup{actions};
  require(!posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO,
                                            path.c_str(),
                                            O_CREAT | O_WRONLY | O_EXCL, 0600),
          "cannot create temporary bag");
  char command[] = "unzip", option[] = "-p";
  char *args[] = {command, option, archive.data(), pattern.data(), nullptr};
  pid_t child;
  require(!posix_spawnp(&child, command, &actions, nullptr, args, environ),
          "cannot run unzip");
  int status;
  pid_t result;
  try {
    do {
      check_running();
      result = waitpid(child, &status, WNOHANG);
      if (result == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } while (result == 0 || (result < 0 && errno == EINTR));
  } catch (...) {
    kill(child, SIGTERM);
    while (waitpid(child, &status, 0) < 0 && errno == EINTR) {
    }
    throw;
  }
  require(result == child && WIFEXITED(status) && !WEXITSTATUS(status),
          "bag extraction failed");
  bag(path, emit);
}
} // namespace recording
