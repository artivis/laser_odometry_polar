#include <laser_odometry_polar/laser_odometry_polar.h>

#include <laser_odometry_core/laser_odometry_utils.h>

#include <pluginlib/class_list_macros.h>

namespace laser_odometry {

// convert from cm to m
constexpr double M_TO_CM = 100.0;
constexpr double CM_TO_M = 1/M_TO_CM;
constexpr double READING_ERROR = 99999;

OdomType LaserOdometryPolar::odomType() const noexcept
{
  return OdomType::Odom2D;
}

bool LaserOdometryPolar::configureImpl()
{
  params_ptr_ = std::make_shared<Parameters>(private_nh_);
  params_ptr_->fromParamServer();

  kf_dist_linear_sq_ = params_ptr_->kf_dist_linear;
  kf_dist_linear_sq_ *= kf_dist_linear_sq_;

  ROS_INFO_STREAM("LaserOdometryPolar parameters:\n" << *params_ptr_);

  return true;
}

bool LaserOdometryPolar::processImpl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                     const Transform& prediction)
{
  std::shared_ptr<PMScan> current_scan_ = std::make_shared<PMScan>(laser_msg->ranges.size());

  convert(laser_msg, current_scan_);

  current_scan_->rx = prediction.translation()(0) * M_TO_CM;
  current_scan_->ry = prediction.translation()(1) * M_TO_CM;
  current_scan_->th = utils::getYaw(prediction.linear());

  prev_scan_->rx = 0;
  prev_scan_->ry = 0;
  prev_scan_->th = 0;

  try
  {
    polar_matcher_.pm_psm(prev_scan_.get(), current_scan_.get());
  }
  catch(const int err)
  {
    ROS_WARN("Error %i in polar scan matching.", err);
    prev_scan_ = current_scan_;
    return false;
  }

  increment_.translation()(0) =  current_scan_->ry * CM_TO_M;
  increment_.translation()(1) = -current_scan_->rx * CM_TO_M;
  increment_.linear() = utils::matrixYaw(current_scan_->th);

  prev_scan_.reset();
  prev_scan_ = current_scan_;
  return true;
}

void LaserOdometryPolar::convert(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                 std::shared_ptr<PMScan>& psm_scan)
{
  psm_scan->rx = 0;
  psm_scan->ry = 0;
  psm_scan->th = 0;

  for (int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    int reading_bad = 0;

    if (scan_msg->ranges[i] == 0)
    {
      psm_scan->r[i] = READING_ERROR;
      reading_bad = 1;
    }
    else
    {
      psm_scan->r[i] = scan_msg->ranges[i] * M_TO_CM;
      psm_scan->x[i] = psm_scan->r[i] * polar_matcher_.pm_co[i];
      psm_scan->y[i] = psm_scan->r[i] * polar_matcher_.pm_si[i];
    }

    psm_scan->bad[i] = reading_bad;
  }

  polar_matcher_.pm_median_filter  (psm_scan.get());
  polar_matcher_.pm_find_far_points(psm_scan.get());
  polar_matcher_.pm_segment_scan   (psm_scan.get());
}

bool LaserOdometryPolar::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  polar_matcher_.PM_L_POINTS         = scan_msg->ranges.size();

  polar_matcher_.PM_FOV              = (scan_msg->angle_max - scan_msg->angle_min) * PM_R2D;
  polar_matcher_.PM_MAX_RANGE        = scan_msg->range_max * M_TO_CM;

  polar_matcher_.PM_TIME_DELAY       = 0.00;

  polar_matcher_.PM_MIN_VALID_POINTS = params_ptr_->min_valid_points;
  polar_matcher_.PM_SEARCH_WINDOW    = params_ptr_->search_window;
  polar_matcher_.PM_MAX_ERROR        = params_ptr_->max_error * M_TO_CM;

  polar_matcher_.PM_MAX_ITER         = params_ptr_->max_iterations;
  polar_matcher_.PM_MAX_ITER_ICP     = params_ptr_->max_iterations;
  polar_matcher_.PM_STOP_COND        = params_ptr_->stop_condition * M_TO_CM;
  polar_matcher_.PM_STOP_COND_ICP    = params_ptr_->stop_condition * M_TO_CM;

  polar_matcher_.pm_init();

  prev_scan_ = std::make_shared<PMScan>(scan_msg->ranges.size());

  return true;
}

bool LaserOdometryPolar::isKeyFrame(const Transform& increment)
{
  if (std::fabs(utils::getYaw(increment.rotation())) > params_ptr_->kf_dist_angular) return true;

  if (increment.translation().head<2>().squaredNorm() > kf_dist_linear_sq_) return true;

  return false;
}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryPolar, laser_odometry::LaserOdometryBase);
