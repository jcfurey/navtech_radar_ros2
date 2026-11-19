// SPDX-License-Identifier: Apache-2.0
#include "dataset_files.hpp"
#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace recording {
TemporaryDirectory::TemporaryDirectory() {
  auto name = (fs::temp_directory_path() / "navtech-recording-XXXXXX").string();
  require(::mkdtemp(name.data()) != nullptr,
          "cannot create temporary recording directory");
  path = name;
}
TemporaryDirectory::~TemporaryDirectory() {
  std::error_code ec;
  fs::remove_all(path, ec);
}
Zip::Zip(const fs::path &path) {
  int error = 0;
  archive_.reset(zip_open(path.c_str(), ZIP_RDONLY, &error));
  require(bool(archive_), "cannot open ZIP " + path.string());
}
std::vector<std::string> Zip::names() const {
  std::vector<std::string> names;
  for (zip_int64_t i = 0; i < zip_get_num_entries(archive_.get(), 0); ++i) {
    auto name = zip_get_name(archive_.get(), i, 0);
    require(name, "invalid ZIP member name");
    names.emplace_back(name);
  }
  return names;
}
Bytes Zip::read(const std::string &name, size_t limit) const {
  zip_stat_t stat{};
  require(zip_stat(archive_.get(), name.c_str(), 0, &stat) == 0 &&
              stat.size <= limit,
          "invalid ZIP member " + name);
  std::unique_ptr<zip_file_t, decltype(&zip_fclose)> file(
      zip_fopen(archive_.get(), name.c_str(), 0), zip_fclose);
  require(bool(file), "cannot open ZIP member " + name);
  Bytes out(stat.size);
  size_t used = 0;
  while (used < out.size()) {
    auto n = zip_fread(file.get(), out.data() + used, out.size() - used);
    require(n > 0, "truncated ZIP member " + name);
    used += n;
  }
  return out;
}
std::string extension(const fs::path &p) {
  auto ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return ext;
}
std::string identify(const fs::path &p) {
  require(fs::exists(p), "missing recording " + p.string());
  auto ext = extension(p);
  if (ext == ".bag")
    return "ros1";
  if (ext == ".cpr" || ext == ".colraw")
    return ext.substr(1);
  if (fs::is_directory(p)) {
    if (p.filename() == "Navtech_Cartesian" &&
        fs::is_regular_file(p.parent_path() / "Navtech_Cartesian.txt"))
      return "radiate_cartesian";
    if (fs::is_regular_file(p / "Navtech_Polar.txt") ||
        (p.filename() == "Navtech_Polar" &&
         fs::is_regular_file(p.parent_path() / "Navtech_Polar.txt")))
      return "radiate";
    if (fs::is_regular_file(p / "metadata.yaml"))
      return "ros2";
  }
  if (ext == ".zip") {
    auto names = Zip(p).names();
    for (const auto &name : names)
      if (extension(name) == ".bag")
        return "ros1";
    for (const auto &name : names) {
      if (fs::path(name).filename() == "metadata.yaml")
        throw std::runtime_error("select an extracted ROS 2 bag directory");
      if (name.rfind("muses/radar/", 0) == 0)
        return "muses";
      if (name.find("/radar/") != name.npos &&
          fs::path(name).extension() == ".png")
        return "oxford";
      if (name.rfind("ray/", 0) == 0)
        return "mulran_ray";
      if (name.rfind("polar/", 0) == 0)
        return "mulran";
    }
  }
  std::string label = p.string();
  std::transform(
      label.begin(), label.end(), label.begin(),
      [](unsigned char c) { return c == '-' ? '_' : std::tolower(c); });
  if (label.find("boreas") != label.npos) {
    if (label.find("road_trip") != label.npos ||
        label.find("boreas_rt") != label.npos)
      return "boreas_rt";
    auto radar = fs::is_directory(p / "radar") ? p / "radar" : p;
    if (fs::is_directory(radar)) {
      uint64_t first = UINT64_MAX;
      for (const auto &f : fs::directory_iterator(radar)) {
        auto stem = f.path().stem().string();
        if (f.path().extension() == ".png" &&
            stem.find_first_not_of("0123456789") == stem.npos)
          first = std::min<uint64_t>(first, std::stoull(stem));
      }
      if (first != UINT64_MAX && first > 1733248400000000ULL)
        return "boreas_rt";
    }
    return "boreas";
  }
  if (label.find("road_trip") != label.npos)
    return "boreas_rt";
  if (label.find("oxford_offroad") != label.npos ||
      p.filename() == "Navtech_CTS350-X_Radar.tar.gz")
    return "oord";
  if (label.find("oxford_radar") != label.npos)
    return "oxford";
  if (label.find("tiny_foggy") != label.npos ||
      label.find("radiate") != label.npos)
    return "radiate";
  if (label.find("muses") != label.npos)
    return "muses";
  if (label.find("mulran") != label.npos)
    return p.filename() == "ray" ? "mulran_ray" : "mulran";
  throw std::runtime_error(
      "unrecognized recording layout; select --format explicitly");
}
} // namespace recording
