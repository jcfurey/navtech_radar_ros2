// SPDX-License-Identifier: Apache-2.0
#include "dataset_files.hpp"
#include <algorithm>
#include <archive.h>
#include <archive_entry.h>
#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <regex>
#include <set>
#include <sstream>

namespace recording {
namespace {
constexpr size_t max_file = 128 * 1024 * 1024;
using Times = std::map<std::string, uint64_t>;
uint64_t number(std::string_view text) {
  uint64_t n{};
  auto r = std::from_chars(text.data(), text.data() + text.size(), n);
  require(r.ec == std::errc{} && r.ptr == text.data() + text.size(),
          "invalid recording integer");
  return n;
}
Bytes read_file(const fs::path &path) {
  auto size = fs::file_size(path);
  require(size <= max_file, "oversized frame " + path.string());
  Bytes data(size);
  std::ifstream f(path, std::ios::binary);
  require(bool(f.read(reinterpret_cast<char *>(data.data()), size)),
          "cannot read " + path.string());
  return data;
}
bool selected(const std::string &profile, const fs::path &p) {
  auto stem = p.stem().string();
  bool numeric =
      !stem.empty() && stem.find_first_not_of("0123456789") == stem.npos;
  if (profile == "mulran_ray")
    return numeric && p.extension() == ".csv";
  if (p.extension() != ".png")
    return false;
  auto part = [&](auto name) {
    return std::find(p.begin(), p.end(), fs::path(name)) != p.end();
  };
  if (profile == "muses")
    return part("radar") && stem.size() >= 6 &&
           stem.substr(stem.size() - 6) == "_radar" && stem.rfind("._", 0) != 0;
  if (profile == "oxford")
    return numeric && part("radar");
  if (profile == "radiate")
    return numeric && part("Navtech_Polar");
  if (profile == "radiate_cartesian")
    return numeric && part("Navtech_Cartesian");
  return numeric;
}
cv::Mat decode(const Bytes &bytes) {
  const uint8_t sig[] = {137, 80, 78, 71, 13, 10, 26, 10};
  require(bytes.size() >= 29 &&
              std::equal(std::begin(sig), std::end(sig), bytes.begin()),
          "invalid radar PNG");
  auto w = big(bytes.data() + 16, 4), h = big(bytes.data() + 20, 4);
  require(w && h && w <= 16400 && h <= 16400 &&
              uint64_t(w) * h <= 2048 * 16400 && bytes[24] == 8 &&
              bytes[25] == 0,
          "unsupported PNG dimensions or encoding");
  auto mat = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
  require(mat.type() == CV_8UC1 && mat.rows == int(h) && mat.cols == int(w),
          "truncated radar PNG");
  return mat;
}
void geometry(Rotation &m, int rows, int bins, double spacing,
              double offset = 0) {
  auto &c = m.configuration;
  c.azimuth_samples = rows;
  c.encoder_size = 5600;
  c.range_bins = bins;
  c.range_resolution = spacing;
  c.bin_size = std::lround(spacing * 10000);
  c.range_gain = 1;
  c.range_offset = offset;
  c.expected_rotation_rate = 4;
  c.sample_bit_depth = 8;
  m.image.width = bins;
  m.image.step = bins;
  m.image.encoding = "mono8";
}
void png_frame(const std::string &p, const Bytes &bytes, uint64_t ns,
               const Emit &emit) {
  cv::Mat img = decode(bytes);
  if (p == "radiate_cartesian") {
    Image out;
    out.height = img.rows;
    out.width = out.step = img.cols;
    out.encoding = "mono8";
    out.data.assign(img.datastart, img.dataend);
    emit(ns, "fft/cartesian", std::move(out));
    return;
  }
  bool frame_only = p == "radiate" || p == "mulran";
  if (frame_only || p == "muses")
    img = img.t();
  int rows = img.rows, width = img.cols - (frame_only ? 0 : 11);
  require(rows <= 2048 && width > 0 && width <= 16384 &&
              (!(frame_only || p == "muses") || rows == 400),
          "invalid FFT frame dimensions");
  Rotation m;
  m.image.height = rows;
  double spacing = 0.0438, offset = 0;
  if (p == "radiate")
    spacing = 0.17361;
  if (p == "mulran")
    spacing = 0.0596;
  if (p == "oxford" || p == "oord") {
    spacing = 0.0432;
    offset = spacing / 2;
  }
  if (p == "muses")
    offset = spacing;
  m.image.data.resize(size_t(rows) * width);
  for (int i = 0; i < rows; ++i) {
    auto row = img.ptr<uint8_t>(i);
    uint16_t enc = uint16_t(std::nearbyint(i * (5600.0 / rows)));
    if (!frame_only) {
      if (p == "muses") {
        uint64_t nano = little(row + 4, 4);
        require(nano < 1000000000, "invalid MUSES subsecond time");
        m.sensor_ns.push_back(little(row, 4) * 1000000000 + nano);
      } else {
        auto us = little(row, 8);
        require(us <= UINT64_MAX / 1000, "invalid radar microseconds");
        m.sensor_ns.push_back(us * 1000);
      }
      if (p == "oord")
        m.sweep_counters.push_back(little(row + 8, 2));
      else
        enc = little(row + 8, 2);
      m.row_flags.push_back(row[10]);
      row += 11;
    }
    require(enc < 5600, "encoder outside a revolution");
    m.encoder_values.push_back(enc);
    std::memcpy(m.image.data.data() + size_t(i) * width, row, width);
  }
  if (!frame_only)
    ns = m.sensor_ns.front();
  if (p == "boreas" || p == "boreas_rt") {
    spacing = ns > 1632182400000000000ULL ? 0.04381 : 0.0596;
    offset = -0.31;
  }
  geometry(m, frame_only ? rows : 400, width, spacing, offset);
  emit(ns, "recording/rotation", std::move(m));
}
void csv_frame(const Bytes &data, const Emit &emit) {
  std::string_view rest(reinterpret_cast<const char *>(data.data()),
                        data.size());
  Rotation m;
  geometry(m, 400, 3360, 0.0596);
  while (!rest.empty()) {
    auto end = rest.find('\n');
    auto row = rest.substr(0, end);
    rest.remove_prefix(end == rest.npos ? rest.size() : end + 1);
    if (!row.empty() && row.back() == '\r')
      row.remove_suffix(1);
    if (row.empty())
      continue;
    auto field = [&]() {
      auto comma = row.find(',');
      auto v = row.substr(0, comma);
      row.remove_prefix(comma == row.npos ? row.size() : comma + 1);
      return v;
    };
    require(m.sensor_ns.size() < 2048, "too many CSV rows");
    m.sensor_ns.push_back(number(field()));
    field();
    auto enc = number(field()), counter = number(field());
    require(enc < 5600 && counter <= 65535, "invalid CSV spoke");
    m.encoder_values.push_back(enc);
    m.sweep_counters.push_back(counter);
    for (int i = 0; i < 3360; ++i) {
      auto n = number(field());
      require(n <= 255, "invalid CSV intensity");
      m.image.data.push_back(n);
    }
    require(row.empty(), "extra CSV bins");
  }
  require(!m.sensor_ns.empty(), "empty CSV frame");
  m.image.height = m.sensor_ns.size();
  auto ns = m.sensor_ns.front();
  emit(ns, "recording/rotation", std::move(m));
}
Times timestamps(const std::string &p, const Bytes &bytes) {
  Times out;
  if (p == "muses") {
    auto meta = nlohmann::json::parse(bytes);
    for (const auto &v : meta) {
      const auto &t = v.at("radar_start_timestamp_us");
      require(t.is_string() || t.is_number_unsigned(),
              "invalid MUSES capture time");
      auto us = t.is_string() ? number(t.get_ref<const std::string &>())
                              : t.get<uint64_t>();
      require(us <= UINT64_MAX / 1000, "MUSES capture time overflow");
      out[fs::path(v.at("path_to_radar").get<std::string>()).stem().string()] =
          us * 1000;
    }
  } else {
    std::string text(bytes.begin(), bytes.end());
    const std::regex pattern(R"(Frame:\s*(\d+)\s+Time:\s*(\d+)(?:\.(\d+))?)");
    for (std::sregex_iterator it(text.begin(), text.end(), pattern), end;
         it != end; ++it) {
      auto frac = (*it)[3].str();
      frac.resize(9, '0');
      auto seconds = number((*it)[2].str());
      require(seconds <= INT32_MAX, "invalid RADIATE time");
      out[(*it)[1].str()] = seconds * 1000000000 + number(frac);
    }
  }
  return out;
}
void unpack_tar(const fs::path &file, const std::string &profile,
                const fs::path &dir) {
  std::unique_ptr<archive, decltype(&archive_read_free)> a(archive_read_new(),
                                                           archive_read_free);
  archive_read_support_filter_gzip(a.get());
  archive_read_support_format_tar(a.get());
  require(archive_read_open_filename(a.get(), file.c_str(), 1 << 16) ==
              ARCHIVE_OK,
          "cannot open tar");
  archive_entry *e;
  int status;
  while ((status = archive_read_next_header(a.get(), &e)) == ARCHIVE_OK) {
    check_running();
    fs::path name = archive_entry_pathname(e);
    if (archive_entry_filetype(e) != AE_IFREG || !selected(profile, name))
      continue;
    auto path = dir / name.filename();
    auto size = archive_entry_size(e);
    require(!fs::exists(path) && size > 0 && uint64_t(size) <= max_file,
            "invalid or duplicate radar tar member");
    std::ofstream out(path, std::ios::binary);
    std::array<char, 65536> buf;
    la_ssize_t n;
    while ((n = archive_read_data(a.get(), buf.data(), buf.size())) > 0) {
      check_running();
      out.write(buf.data(), n);
    }
    require(!n && bool(out), "cannot stage radar tar member");
  }
  require(status == ARCHIVE_EOF, "truncated tar archive");
}
} // namespace
void read_dataset(const fs::path &path, const std::string &p,
                  const Emit &emit) {
  static const std::set<std::string> profiles = {
      "oord",       "oxford", "boreas",  "boreas_rt",        "mulran",
      "mulran_ray", "muses",  "radiate", "radiate_cartesian"};
  require(profiles.count(p), "unknown file playback format " + p);
  if (path.extension() == ".gz") {
    TemporaryDirectory tmp;
    unpack_tar(path, p, tmp.path);
    read_dataset(tmp.path, p, emit);
    return;
  }
  std::unique_ptr<Zip> zip;
  std::vector<std::string> names;
  if (fs::is_regular_file(path)) {
    zip = std::make_unique<Zip>(path);
    names = zip->names();
  } else {
    auto root = fs::absolute(path);
    if ((p == "boreas" || p == "boreas_rt") && fs::is_directory(root / "radar"))
      root /= "radar";
    for (const auto &f : fs::recursive_directory_iterator(root))
      if (f.is_regular_file())
        names.push_back(f.path().string());
  }
  auto read = [&](const auto &n) {
    return zip ? zip->read(n, max_file) : read_file(n);
  };
  Times times;
  bool timed = p == "radiate" || p == "radiate_cartesian" || p == "muses";
  if (timed) {
    std::string table = p == "muses"     ? "meta.json"
                        : p == "radiate" ? "Navtech_Polar.txt"
                                         : "Navtech_Cartesian.txt";
    fs::path found;
    if (!zip) {
      auto dir = fs::absolute(path);
      for (int i = 0; i < 3; ++i, dir = dir.parent_path())
        if (fs::is_regular_file(dir / table)) {
          found = dir / table;
          break;
        }
    }
    if (found.empty()) {
      for (const auto &name : names)
        if (fs::path(name).filename() == table) {
          require(found.empty(), "ambiguous " + table);
          found = name;
        }
    }
    require(!found.empty(), "missing " + table);
    times = timestamps(p, read(found.string()));
  }
  std::vector<std::pair<uint64_t, std::string>> frames;
  for (const auto &name : names)
    if (selected(p, name)) {
      auto stem = fs::path(name).stem().string();
      frames.emplace_back(timed ? times.at(stem) : number(stem), name);
    }
  std::sort(frames.begin(), frames.end());
  for (const auto &[ns, name] : frames) {
    check_running();
    auto bytes = read(name);
    if (p == "mulran_ray")
      csv_frame(bytes, emit);
    else
      png_frame(p, bytes, ns, emit);
  }
}
} // namespace recording
