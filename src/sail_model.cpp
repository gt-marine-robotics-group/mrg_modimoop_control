#include "sail_model.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace sailbot_control
{

TableSailModel::TableSailModel(
  std::vector<double> alpha_deg,
  std::vector<double> L_main,
  std::vector<double> D_main,
  std::vector<double> L_back,
  std::vector<double> D_back)
: alpha_deg_(std::move(alpha_deg)),
  L_main_(std::move(L_main)),
  D_main_(std::move(D_main)),
  L_back_(std::move(L_back)),
  D_back_(std::move(D_back))
{
  validate();
}

std::string TableSailModel::trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

std::vector<std::string> TableSailModel::splitCsvLine(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(trim(item));
  }
  return out;
}

std::size_t TableSailModel::requireColumn(
  const std::vector<std::string> & header,
  const std::string & name)
{
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (header[i] == name) {
      return i;
    }
  }
  throw std::runtime_error("Missing required CSV column: " + name);
}

TableSailModel TableSailModel::fromCsv(const std::string & csv_path)
{
  std::ifstream in(csv_path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open sail table CSV: " + csv_path);
  }

  std::string line;
  if (!std::getline(in, line)) {
    throw std::runtime_error("CSV is empty: " + csv_path);
  }

  const auto header = splitCsvLine(line);
  const std::size_t i_alpha = requireColumn(header, "alpha_deg");
  const std::size_t i_Lm = requireColumn(header, "Liftm");
  const std::size_t i_Dm = requireColumn(header, "Dragm");
  const std::size_t i_Lt = requireColumn(header, "Liftt");
  const std::size_t i_Dt = requireColumn(header, "Dragt");

  std::vector<double> alpha_deg;
  std::vector<double> L_main;
  std::vector<double> D_main;
  std::vector<double> L_back;
  std::vector<double> D_back;

  while (std::getline(in, line)) {
    if (trim(line).empty()) {
      continue;
    }

    const auto row = splitCsvLine(line);
    const auto need = std::max({i_alpha, i_Lm, i_Dm, i_Lt, i_Dt});
    if (row.size() <= need) {
      throw std::runtime_error("Malformed CSV row in: " + csv_path);
    }

    alpha_deg.push_back(std::stod(row[i_alpha]));
    L_main.push_back(std::stod(row[i_Lm]));
    D_main.push_back(std::stod(row[i_Dm]));
    L_back.push_back(std::stod(row[i_Lt]));
    D_back.push_back(std::stod(row[i_Dt]));
  }

  return TableSailModel(
    std::move(alpha_deg),
    std::move(L_main),
    std::move(D_main),
    std::move(L_back),
    std::move(D_back));
}

void TableSailModel::validate() const
{
  const std::size_t n = alpha_deg_.size();
  if (n < 2) {
    throw std::runtime_error("Sail table needs at least 2 alpha points");
  }
  if (L_main_.size() != n || D_main_.size() != n ||
      L_back_.size() != n || D_back_.size() != n) {
    throw std::runtime_error("Sail table vectors are mismatched in size");
  }

  for (std::size_t i = 1; i < n; ++i) {
    if (!(alpha_deg_[i] > alpha_deg_[i - 1])) {
      throw std::runtime_error("alpha_deg must be strictly increasing");
    }
  }
}

double TableSailModel::interpExtrap(
  const std::vector<double> & xs,
  const std::vector<double> & ys,
  double x)
{
  if (xs.size() < 2 || ys.size() != xs.size()) {
    throw std::runtime_error("interpExtrap received invalid table vectors");
  }

  auto lerp = [&](std::size_t i0, std::size_t i1) {
    const double x0 = xs[i0];
    const double x1 = xs[i1];
    const double y0 = ys[i0];
    const double y1 = ys[i1];
    const double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  };

  if (x <= xs.front()) {
    return lerp(0, 1);
  }
  if (x >= xs.back()) {
    return lerp(xs.size() - 2, xs.size() - 1);
  }

  const auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const std::size_t i1 = static_cast<std::size_t>(std::distance(xs.begin(), it));
  const std::size_t i0 = i1 - 1;
  return lerp(i0, i1);
}

SailForces TableSailModel::main(double alpha_deg) const
{
  return SailForces{
    interpExtrap(alpha_deg_, L_main_, alpha_deg),
    interpExtrap(alpha_deg_, D_main_, alpha_deg)
  };
}

SailForces TableSailModel::back(double alpha_deg) const
{
  return SailForces{
    interpExtrap(alpha_deg_, L_back_, alpha_deg),
    interpExtrap(alpha_deg_, D_back_, alpha_deg)
  };
}

}  // namespace sailbot_control