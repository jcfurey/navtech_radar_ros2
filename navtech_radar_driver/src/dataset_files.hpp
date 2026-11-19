// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "recording.hpp"
#include <memory>
#include <zip.h>

namespace recording {
std::string extension(const fs::path &);
// The ZIP central directory permits chronological reads without extracting a
// dataset.
class Zip {
public:
  explicit Zip(const fs::path &path);
  std::vector<std::string> names() const;
  Bytes read(const std::string &name, size_t limit) const;

private:
  std::unique_ptr<zip_t, decltype(&zip_discard)> archive_{nullptr, zip_discard};
};
} // namespace recording
