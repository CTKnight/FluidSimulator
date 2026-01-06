#include "fluid/vtk_writer.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace fluid {
namespace {

bool ensure_dir(const std::string& path) {
  std::error_code ec;
  if (path.empty()) {
    return false;
  }
  std::filesystem::create_directories(path, ec);
  return !ec;
}

}  // namespace

VtkWriter::VtkWriter(std::string output_dir, std::string basename)
    : output_dir_(std::move(output_dir)), basename_(std::move(basename)) {}

bool VtkWriter::write_frame(const VtkFrameView& frame,
                            std::size_t frame_index,
                            std::string* out_path) const {
  if (!frame.pos_x || !frame.pos_y || !frame.pos_z) {
    return false;
  }
  if (!ensure_dir(output_dir_)) {
    return false;
  }
  const std::string filename = make_frame_filename(basename_, frame_index);
  const std::string path = join_path(output_dir_, filename);
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) {
    return false;
  }

  out << "# vtk DataFile Version 3.0\n";
  out << "FluidSimulator frame " << frame_index << "\n";
  out << "ASCII\n";
  out << "DATASET POLYDATA\n";
  out << "POINTS " << frame.count << " float\n";
  out << std::setprecision(9) << std::fixed;
  for (std::size_t i = 0; i < frame.count; ++i) {
    out << frame.pos_x[i] << ' ' << frame.pos_y[i] << ' ' << frame.pos_z[i]
        << '\n';
  }
  out << "VERTICES " << frame.count << ' ' << frame.count * 2 << "\n";
  for (std::size_t i = 0; i < frame.count; ++i) {
    out << "1 " << i << '\n';
  }

  if (!out) {
    return false;
  }
  if (out_path) {
    *out_path = path;
  }
  return true;
}

std::string VtkWriter::make_frame_filename(const std::string& basename,
                                           std::size_t frame_index) {
  std::ostringstream name;
  name << basename << '_' << std::setfill('0') << std::setw(6) << frame_index
       << ".vtk";
  return name.str();
}

std::string VtkWriter::join_path(const std::string& dir,
                                 const std::string& file) {
  if (dir.empty()) {
    return file;
  }
  if (dir.back() == '/' || dir.back() == '\\') {
    return dir + file;
  }
  return dir + '/' + file;
}

PvdWriter::PvdWriter(std::string output_dir, std::string basename)
    : output_dir_(std::move(output_dir)), basename_(std::move(basename)) {}

void PvdWriter::add_frame(double time, const std::string& relative_path) {
  entries_.push_back(Entry{time, relative_path});
}

bool PvdWriter::write() const {
  if (!ensure_dir(output_dir_)) {
    return false;
  }
  const std::string path = pvd_path();
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) {
    return false;
  }

  out << "<?xml version=\"1.0\"?>\n";
  out << "<VTKFile type=\"Collection\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
  out << "  <Collection>\n";
  out << std::setprecision(9) << std::fixed;
  for (const auto& entry : entries_) {
    out << "    <DataSet timestep=\"" << entry.time
        << "\" group=\"\" part=\"0\" file=\"" << entry.file
        << "\"/>\n";
  }
  out << "  </Collection>\n";
  out << "</VTKFile>\n";

  return static_cast<bool>(out);
}

std::string PvdWriter::pvd_path() const {
  return VtkWriter::join_path(output_dir_, basename_ + ".pvd");
}

}  // namespace fluid
