#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace fluid {

struct VtkFrameView {
  const float* pos_x = nullptr;
  const float* pos_y = nullptr;
  const float* pos_z = nullptr;
  std::size_t count = 0;
  double time = 0.0;
};

class VtkWriter {
 public:
  VtkWriter(std::string output_dir, std::string basename = "frame");

  bool write_frame(const VtkFrameView& frame,
                   std::size_t frame_index,
                   std::string* out_path = nullptr) const;

  static std::string make_frame_filename(const std::string& basename,
                                         std::size_t frame_index);
  static std::string join_path(const std::string& dir,
                               const std::string& file);

 private:
  std::string output_dir_;
  std::string basename_;
};

class PvdWriter {
 public:
  PvdWriter(std::string output_dir, std::string basename = "series");

  void add_frame(double time, const std::string& relative_path);
  bool write() const;
  std::string pvd_path() const;

 private:
  struct Entry {
    double time = 0.0;
    std::string file;
  };

  std::string output_dir_;
  std::string basename_;
  std::vector<Entry> entries_;
};

}  // namespace fluid
