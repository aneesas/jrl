#include "jrl/IOMeasurements.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_measurements {

/**********************************************************************************************************************/
gtsam::Matrix parseCovariance(json input_json, int d) {
  auto v = input_json.get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), d, d);
  return m;
}

gtsam::Matrix parseCovarianceAsInformation(json input_json, int d) {
  auto v = input_json.get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), d, d);
  gtsam::Matrix I = m.inverse();  // full information matrix = inverse of covariance
  gtsam::Matrix I_kt = Eigen::MatrixXd::Zero(d, d);
  // Compute precisions kappa and tau from I, which we assume to be symmetric
  // The ordering of values in covariance (and in information) depends on dimension
  const double I11 = I(0, 0);
  const double I12 = I(0, 1);
  const double I13 = I(0, 2);
  const double I22 = I(1, 1);
  const double I23 = I(1, 2);
  const double I33 = I(2, 2);

  if (d == 6) {
    // Pose3 ordered as (R, t)
    // Parse the rotation components
    Eigen::Matrix3d rot;
    rot << I11, I12, I13, I12, I22, I23, I13, I23, I33;
    double kappa = 3 / (2 * rot.inverse().trace());

    // Parse the additional translation values
    const double I44 = I(3, 3);
    const double I45 = I(3, 4);
    const double I46 = I(3, 5);
    const double I55 = I(4, 4);
    const double I56 = I(4, 5);
    const double I66 = I(5, 5);
    Eigen::Matrix3d tran;
    tran << I44, I45, I46, I45, I55, I56, I46, I56, I66;
    double tau = 3 / tran.inverse().trace();

    // Fill in I_kt
    I_kt(0,0) = kappa;
    I_kt(1,1) = kappa;
    I_kt(2,2) = kappa;
    I_kt(3,3) = tau;
    I_kt(4,4) = tau;
    I_kt(5,5) = tau;

  } else if (d == 3) {
    // Pose2 ordered as (x, y, theta)
    Eigen::Matrix2d tran;
    tran << I11, I12, I12, I22;
    double tau = 2 / tran.inverse().trace();
    double kappa = I33;

    // Fill in I_kt
    I_kt(0,0) = tau;
    I_kt(1,1) = tau;
    I_kt(2,2) = kappa;

  } else {
    std::cout << "ERROR: parseCovarianceAsInformation called with unsupported dimension" << std::endl;

  }

  return I_kt;
}

json serializeCovariance(gtsam::Matrix covariance) {
  std::vector<double> vec(covariance.data(), covariance.data() + covariance.rows() * covariance.cols());
  return json(vec);
}

}  // namespace io_measurements
}  // namespace jrl
