#ifndef RBL_CONTROLLER_H
#define RBL_CONTROLLER_H

// CUSTOM
#include "rbl_replanner/replanner.h"
#include "ciri/ciri.h"

// MRS

// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/Point.h>
// #include <mrs_msgs/msg/control_manager_diagnostics.hpp>
// #include <mrs_msgs/msg/float64_stamped.hpp>
// #include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/msg/reference.hpp>
// #include <std_msgs/String.h>
// #include <std_srvs/Trigger.h>
// #include <mrs_msgs/Vec4.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <mrs_lib/geometry/misc.h>
// #include <mrs_lib/transformer.h>
// #include <mrs_lib/scope_timer.h>
// #include "rbl_controller/ActivateParams.h"

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Eigen>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

// Standard CPP libs
#include <stdio.h>
#include <filesystem>
#include <boost/make_shared.hpp>
#include <queue>
#include <unordered_map>
#include <tuple>
#include <string>
#include <vector>
#include <set>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>
#include <deque>
#include <utility>
#include <memory>
#include <chrono>
#include <optional>
#include <mutex>
#include <future>
#include <cstdint>



struct RBLParams {
  double                                step_size;
  double                                radius;
  double                                path_lookahead_distance        = 0.0;
  double                                encumbrance;
  double                                dt;
  double                                beta_min;
  double                                betaD;
  double                                d1;
  double                                d2;
  double                                d3;
  double                                d4;
  double                                d5;
  double                                d6;
  double                                d7;
  double                                cwvd_rob;
  double                                cwvd_obs;
  bool                                  use_z_rule;
  double                                z_min;
  double                                z_max;
  double                                boundary_threshold;
  double                                boundary_threshold_speed;
  double                                lidar_tilt;
  double                                lidar_fov;
  bool                                  move_centroid_to_sensed_cell;
  bool                                  use_garmin_alt;
  bool                                  only_2d                       = false;
  double                                z_ref                         = 1.0;
  bool                                  use_map                       = true;
  double                                voxel_size                    = 0.4; //discretization for faster processing of obstacles for raw pcl and also needs to be set if map is used
  bool                                  replanner                     = false; //if true the alg also needs garmin alt - ground truth. For replanner map does not map bellow uav at the start;
  bool                                  limited_fov                   = true;
  bool                                  ciri                          = false;
  bool                                  add_estimates_as_voxels       = true;
  double                                inflation_bonus               = 0.0;
    bool downsample_pcl = false;
};

struct State {
Eigen::Vector3d position;
Eigen::Vector3d velocity;
};

class RBLController {
public:
  RBLController(const RBLParams& par);
  void setCurrentPosition(const Eigen::Vector3d& point);
  void setCurrentVelocity(const Eigen::Vector3d& point);
  void setGroupStates(const std::vector<State>& states);
    void setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud);
    void setPCL1(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud);
  void setGoal(const Eigen::Vector3d& point);
  void setBetaD(double beta);
  void setAltitude(const double& alt);
  void setRollPitchYaw(const Eigen::Vector3d& rpy);


bool inputsHealthy( const Eigen::Vector3d&                                            agent_pos, 
                    const Eigen::Vector3d&                                            agent_vel, 
                    const std::vector<State>&   group_states,
                    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>&                  cloud, 
                    Eigen::Vector3d&                                                  goal, 
                    double&                                                           altitude, 
                    Eigen::Vector3d&                                                  rpy);

  std::optional<mrs_msgs::msg::Reference>            getNextRef();
  Eigen::Vector3d                               getGoal(); 
  Eigen::Vector3d                               getWaypoint();
  Eigen::Vector3d                               getCurrentPosition();
  Eigen::Vector3d                               getCurrentVelocity();
  Eigen::Vector3d                               getCentroid();
  Eigen::Vector3d                               getSeedB();
  std::vector<Eigen::Vector3d>                  getCellA();
  std::vector<Eigen::Vector3d>                  getSensedCellA();
  std::vector<Eigen::Vector3d>                  getCellS();
  std::vector<Eigen::Vector3d>                  getLocalObstaclePoints();
  std::vector<Eigen::Vector3d>                  getInflatedMap();
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> getPCL();
  std::vector<Eigen::Vector3d>                  getPath();

private:
  RBLParams                                                 params_;
  bool                                                      flag_threshold;
  bool                                                      init_ = false;
  bool                                                      has_goal_ = false;
  bool                                                      pending_replan_ = false;
  bool                                                      threshold_active_ = false;
  double                                                    radius_sensing_;
  double                                                    altitude_;
  double                                                    beta_;
  double                                                    ph_; //vertical
  double                                                    th_; //azimuthal
  Eigen::Vector3d                                           goal_ = Eigen::Vector3d::Zero(); //final goal where the uav will converge
  Eigen::Vector3d                                           destination_ = Eigen::Vector3d::Zero(); //rotated current goal/waypoint
  Eigen::Vector3d                                           waypoint_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d                                           seed_b_= Eigen::Vector3d::Zero();
  Eigen::Vector3d                                           waypoint_fixed_distance_ = Eigen::Vector3d::Zero(); //replanner waypoint
  Eigen::Vector3d                                           agent_pos_ = Eigen::Vector3d::Zero(); 
  Eigen::Vector3d                                           agent_vel_ = Eigen::Vector3d::Zero(); 
  Eigen::Vector3d                                           rpy_ = Eigen::Vector3d::Zero(); 
  Eigen::Vector3d                                           c1_= Eigen::Vector3d::Zero();
  Eigen::Vector3d                                           c1_full_= Eigen::Vector3d::Zero();
  Eigen::Vector3d                                           c2_;
  Eigen::Vector3d                                           c1_no_rot_;
  std::vector<State>                              group_states_;
  std::vector<Eigen::Vector3d>                              cell_A_;
  std::vector<Eigen::Vector3d>                              sensed_cell_A_;
  std::vector<Eigen::Vector3d>                              cell_S_;
  std::vector<Eigen::Vector3d>                              local_obstacle_points_;
  std::vector<Eigen::Vector3d>                              plane_normals_;
  std::vector<Eigen::Vector3d>                              plane_points_;
  std::vector<Eigen::Vector3d>                              inflated_map_;
  std::vector<Eigen::Vector3d>                              injected_points_map_;
  std::vector<Eigen::Vector3d>                              path_;
  std::size_t                                               path_progress_index_ = 0;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>           cloud_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>           cloud_obs_;
  std::shared_ptr<RBLReplanner>                             rbl_replanner_;
  std::shared_ptr<CIRI>                                     ciri_solver_;
  std::uint64_t                                             goal_generation_ = 0;
  std::future<std::tuple<std::uint64_t, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> replanner_future_;
  std::mutex                                                replanner_mutex_;

  // Recovery/stuck detection: controller-side state for escaping repeat local minima.
  // The real user goal stays in goal_; during ESCAPE_BACKTRACK only active_recovery_goal_
  // is sent to the replanner, then normal planning to goal_ resumes after cooldown.
  enum class RecoveryMode {
    NORMAL,
    ESCAPE_BACKTRACK,
    RECOVERY_COOLDOWN,
  };

  struct Breadcrumb {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    std::chrono::steady_clock::time_point stamp;
    std::uint64_t goal_generation = 0;
  };

  struct VirtualObstacleMemory {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double radius = 0.0;
    double weight = 0.0;
    int repeat_count = 0;
    std::chrono::steady_clock::time_point expires_at;
  };

  RecoveryMode                                             recovery_mode_ = RecoveryMode::NORMAL;
  std::deque<Breadcrumb>                                   breadcrumbs_;
  std::vector<VirtualObstacleMemory>                       virtual_obstacles_;
  Eigen::Vector3d                                          active_recovery_goal_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d                                          recovery_last_progress_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d                                          recovery_last_stuck_pos_ = Eigen::Vector3d::Zero();
  double                                                   recovery_best_goal_dist_ = std::numeric_limits<double>::infinity();
  int                                                      recovery_repeat_stuck_count_ = 0;
  bool                                                     recovery_has_progress_sample_ = false;
  bool                                                     recovery_has_stuck_sample_ = false;
  std::chrono::steady_clock::time_point                    recovery_goal_set_time_;
  std::chrono::steady_clock::time_point                    recovery_last_progress_time_;
  std::chrono::steady_clock::time_point                    recovery_escape_started_time_;
  std::chrono::steady_clock::time_point                    recovery_cooldown_until_;
  std::chrono::steady_clock::time_point                    path_last_accept_time_;
  RecoveryMode                                             accepted_path_recovery_mode_ = RecoveryMode::NORMAL;
  Eigen::Vector3d                                          accepted_path_goal_ = Eigen::Vector3d::Zero();

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> getGroundCleanCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud, const Eigen::Vector3d& agent_pos, const double& altitude);
std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> downSamplePcl(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,  // //{
                                double                                           voxel_size);
  std::vector<Eigen::Vector3d> getpointsInsideCircle(const Eigen::Vector3d& center, const double& radius, const double& step_size);
  void pointsInsideSphere(std::vector<Eigen::Vector3d>& sphere, const Eigen::Vector3d& center, const double& radius, const double& step_size, const double& altitude);
  void partitionCellA(std::vector<Eigen::Vector3d>&                             cell_A, 
                      std::vector<Eigen::Vector3d>&                             cell_S, 
                      std::vector<Eigen::Vector3d>&                             plane_normals,
                      std::vector<Eigen::Vector3d>&                             plane_points,
                      const Eigen::Vector3d&                                    agent_pos,
                      const std::vector<Eigen::Vector3d>&                       neighbors,
                      std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>&          cloud);
  bool partitionCellACiri(std::vector<Eigen::Vector3d>&                    cell_A,
                          std::vector<Eigen::Vector3d>&                    cell_S,
                          std::vector<Eigen::Vector3d>&                    plane_normals,
                          std::vector<Eigen::Vector3d>&                    plane_points,
                          const Eigen::Vector3d&                           agent_pos,
                          const Eigen::Vector3d&                           waypoint,
                          const std::vector<Eigen::Vector3d>&              neighbors,
                          std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                          Eigen::Vector3d&                                 c1,
                          Eigen::Vector3d&                                 seed_b,
                          bool&                                            threshold_active);
  void convertPlaneData(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& plane_data, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points, const Eigen::Vector3d& agent_pos);
  void closestPointOnVoxel(Eigen::Vector3d& point, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& voxel_center, const double& voxel_size);
  void createAndPartitionCellA(std::vector<Eigen::Vector3d>& cell_A, std::vector<Eigen::Vector3d>& sensed_cell_A_, std::vector<Eigen::Vector3d>& cell_S, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& rpy, const Eigen::Vector3d& waypoint, const std::vector<Eigen::Vector3d>& neighbors_pos, std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud, const double& altitude, Eigen::Vector3d& c1, Eigen::Vector3d& seed_b, bool& threshold_active );
  std::vector<Eigen::Vector3d> computeActivelySensedCell(std::vector<Eigen::Vector3d>& cell_A, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& rpy);
  Eigen::Matrix3d Rx(double angle);
  Eigen::Matrix3d Ry(double angle);
  Eigen::Matrix3d Rz(double angle);
  Eigen::Vector3d movePointToCell(const Eigen::Vector3d& point, const std::vector<Eigen::Vector3d>& cell);
  void computeCentroid(Eigen::Vector3d& centroid, Eigen::Vector3d& agent_pos, Eigen::Vector3d& agent_vel, std::vector<Eigen::Vector3d>& cell, std::vector<Eigen::Vector3d>& plane_normals, std::vector<Eigen::Vector3d>& plane_points, Eigen::Vector3d& destination, Eigen::Vector3d& goal, double& beta, bool flag_threshold, bool& threshold_active_);
  void computeScalarValue(std::vector<double>& scalar_values, const std::vector<double>& x_test, const std::vector<double>& y_test, const std::vector<double>& z_test, const Eigen::Vector3d &destination, const Eigen::Vector3d &goal, double beta);
  void applyRules(double& beta, double& th, double& ph, Eigen::Vector3d destination, Eigen::Vector3d& seed_b, 
                  const Eigen::Vector3d goal, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& c1, const Eigen::Vector3d& c2, const Eigen::Vector3d& c1_no_rot,
                  const double& d1, const double& d2, const double& d3, const double& d4, const double& d5, const double& d6, const double& d7, const double& betaD, const double& beta_min, const double& dt);
  Eigen::Vector3d determineWaypoint(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& goal, Eigen::Vector3d& waypoint);
  Eigen::Vector3d determineWaypointFixedDistance(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& goal);
  bool isWaypointSegmentClear(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
  bool shouldAcceptReplannerPath(const std::vector<Eigen::Vector3d>& new_path, const Eigen::Vector3d& planning_goal) const;
  void acceptReplannerPath(std::vector<Eigen::Vector3d>&& new_path, std::vector<Eigen::Vector3d>&& new_inflated_map, const Eigen::Vector3d& planning_goal);
  void clearReplannerPath();
  void determineNextRef(mrs_msgs::msg::Reference& p_ref, const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& waypoint, const Eigen::Vector3d& goal, const Eigen::Vector3d& c1, const Eigen::Vector3d& c1_full, const Eigen::Vector3d& rpy, const std::vector<Eigen::Vector3d>& path);
  mrs_msgs::msg::Reference recoveryFallbackRef() const;
  void updateRecoveryState();
  void resetRecoveryForNewGoal();
  void appendRecoveryBreadcrumb(const std::chrono::steady_clock::time_point& now);
  bool selectEscapeBreadcrumb(Eigen::Vector3d& escape_goal) const;
  void enterRecoveryEscape(const std::chrono::steady_clock::time_point& now);
  void addOrStrengthenVirtualObstacle(const Eigen::Vector3d& stuck_position, const std::chrono::steady_clock::time_point& now);
  void addVirtualObstacleMemory(const Eigen::Vector3d& center, double radius, double weight, const std::chrono::steady_clock::time_point& now);
  void addFailedPathVirtualObstacles(const Eigen::Vector3d& stuck_position, const Eigen::Vector3d& escape_goal, const std::chrono::steady_clock::time_point& now);
  void pruneVirtualObstacles(const std::chrono::steady_clock::time_point& now);
  std::vector<ReplannerVirtualObstacle> activeVirtualObstacles(const std::chrono::steady_clock::time_point& now) const;
  Eigen::Vector3d activePlanningGoal() const;
  mrs_msgs::msg::Reference pRefAgent(const Eigen::Vector3d& agent_pos, const double yaw);
  double determineYaw(const Eigen::Vector3d& agent_pos, const Eigen::Vector3d& waypoint, const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& rpy);
  // double determineYaw(const Eigen::Vector3d& agent_pos, const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& rpy);
  double normalizeAngle(double angle);
};

#endif
