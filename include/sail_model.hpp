#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace sailbot_control
{

struct SailForces
{
  double L{0.0};
  double D{0.0};
};

class SailModel
{
public:
  virtual ~SailModel() = default;
  virtual SailForces main(double alpha_deg) const = 0;
  virtual SailForces back(double alpha_deg) const = 0;
};

class TableSailModel final : public SailModel
{
public:
  TableSailModel(
    std::vector<double> alpha_deg,
    std::vector<double> L_main,
    std::vector<double> D_main,
    std::vector<double> L_back,
    std::vector<double> D_back);

  static TableSailModel fromCsv(const std::string & csv_path);

  SailForces main(double alpha_deg) const override;
  SailForces back(double alpha_deg) const override;

private:
  void validate() const;

  static double interpExtrap(
    const std::vector<double> & xs,
    const std::vector<double> & ys,
    double x);

  static std::vector<std::string> splitCsvLine(const std::string & line);
  static std::string trim(const std::string & s);
  static std::size_t requireColumn(
    const std::vector<std::string> & header,
    const std::string & name);

  std::vector<double> alpha_deg_;
  std::vector<double> L_main_;
  std::vector<double> D_main_;
  std::vector<double> L_back_;
  std::vector<double> D_back_;
};

}  // namespace sailbot_control