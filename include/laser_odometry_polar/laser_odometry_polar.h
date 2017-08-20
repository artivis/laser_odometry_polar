#ifndef _LASER_ODOMETRY_POLAR_LASER_ODOMETRY_POLAR_H_
#define _LASER_ODOMETRY_POLAR_LASER_ODOMETRY_POLAR_H_

#include <laser_odometry_core/laser_odometry_core.h>

#include <polar_scan_matcher/polar_match.h>

#include <laser_odometry_polar/LaserOdometryPolarParameters.h>

namespace laser_odometry
{

class LaserOdometryPolar : public LaserOdometryBase
{
  using Base = LaserOdometryBase;

  using Parameters = laser_odometry_polar::LaserOdometryPolarParameters;
  using ParametersPtr = std::shared_ptr<Parameters>;

public:

  LaserOdometryPolar()  = default;
  ~LaserOdometryPolar() = default;

  OdomType odomType() const noexcept override;

protected:

  bool processImpl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                   const Transform& prediction) override;

protected:

  ParametersPtr params_ptr_;
  double kf_dist_linear_sq_;

  std::shared_ptr<PMScan> prev_scan_;

  PolarMatcher polar_matcher_;

  void convert(const sensor_msgs::LaserScanConstPtr& scan_msg,
               std::shared_ptr<PMScan>& psm_scan);

  bool configureImpl() override;

  bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;

  void updateLaserPose();

  bool isKeyFrame(const Transform& increment) override;
};

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_POLAR_LASER_ODOMETRY_POLAR_H_ */
