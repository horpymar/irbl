#include "rbl_replanner/replanner.h"

RBLReplanner::RBLReplanner(const ReplannerParams& params) : params_(params)  // //{
{
  std::cout << "[RBLReplanner]: Replanner initialization" << std::endl;
  voxel_size_ = roundToNextMultiple(params.voxel_size, params.replanner_vox_size);
  inflation_  = roundToNextMultiple(params.encumbrance + params.inflation_bonus, params.replanner_vox_size);
  // std::cout << "[RBLReplanner]: voxel_size_: " << voxel_size_ << ", inflation_: " << inflation_ << std::endl;
  // A*/RBL consensus: keep A* hard inflation at least as conservative as the controller safety radius.
  inflation_coeff_ = std::ceil(inflation_ / params.replanner_vox_size);
  std::cout << "Inflation coef: " << inflation_coeff_ << std::endl;
  //   int inflation_coeff = std::ceil(encumbrance / map_resolution);

  _X_ = static_cast<int>(std::ceil(params.map_width / params.replanner_vox_size));
  _Y_ = static_cast<int>(std::ceil(params.map_width / params.replanner_vox_size));
  _Z_ = static_cast<int>(std::ceil(params.map_height / params.replanner_vox_size));
  // Make them even
  _X_ = (_X_ % 2 == 0) ? _X_ : _X_ + 1;
  _Y_ = (_Y_ % 2 == 0) ? _Y_ : _Y_ + 1;
  _Z_ = (_Z_ % 2 == 0) ? _Z_ : _Z_ + 1;

  replanner_period_ = 1.0 / params.replanner_freq;
  first_plan        = true;
  goal_changed_     = false;

  _inflated_grid_  = VoxelGrid(_X_, _Y_, _Z_);
  _clearance_grid_ = VoxelGrid(_X_, _Y_, _Z_);
}  // //}

void RBLReplanner::setCurrentPosition(const Eigen::Vector3d& point)  // //{
{
  agent_pos_ = point;
  // std::cout << "[RBLReplanner] Setting Agent position to x: " << agent_pos_.x() << ", y: " << agent_pos_.y() << ", z:
  // " << agent_pos_.z() << std::endl;
}  // //}

void RBLReplanner::setGoal(const Eigen::Vector3d& point)  // //{
{
  double tolerance = 1e-6;
  if (!goal_.isApprox(point, tolerance)) {
    goal_changed_ = true;
    goal_         = point;
  }
}  // //}

void RBLReplanner::setAltitude(const double& alt)  // //{
{
  altitude_ = alt;
}  // //}

void RBLReplanner::setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  cloud_ = cloud;
}  // //}

void RBLReplanner::setVirtualObstacles(const std::vector<ReplannerVirtualObstacle>& obstacles)  // //{
{
  // Recovery/stuck detection: virtual obstacles are a soft memory of repeated local minima.
  // They are not written into the occupancy grid, so they can expire without corrupting the map.
  const bool obstacle_count_changed = virtual_obstacles_.size() != obstacles.size();
  bool obstacle_values_changed = obstacle_count_changed;
  if (!obstacle_values_changed) {
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
      if (!virtual_obstacles_[i].center.isApprox(obstacles[i].center, 1e-6) ||
          std::abs(virtual_obstacles_[i].radius - obstacles[i].radius) > 1e-6 ||
          std::abs(virtual_obstacles_[i].weight - obstacles[i].weight) > 1e-6) {
        obstacle_values_changed = true;
        break;
      }
    }
  }

  virtual_obstacles_ = obstacles;
  if (obstacle_values_changed) {
    goal_changed_ = true;
  }
}  // //}

std::vector<Eigen::Vector3d> RBLReplanner::getInflatedCloud()  // //{
{
  const auto start = std::chrono::steady_clock::now();
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < _inflated_grid_->X; ++x) {
    for (int y = 0; y < _inflated_grid_->Y; ++y) {
      for (int z = 0; z < _inflated_grid_->Z; ++z) {
        if (_inflated_grid_->at(x, y, z) == 1) {
          points.push_back(gridIdxToWorldCoords(x, y, z));
        }
      }
    }
  }
  // std::cout << "[RBLReplanner]: getting inflated cloud of size: " << points.size() << std::endl;
  const auto stop = std::chrono::steady_clock::now();
  std::cout << "[RBLReplanner][timing] getInflatedCloud total_ms="
            << std::chrono::duration<double, std::milli>(stop - start).count()
            << ", points=" << points.size()
            << ", grid=[" << _X_ << "," << _Y_ << "," << _Z_ << "]" << std::endl;
  return points;
}  // //}

std::vector<Eigen::Vector3d> RBLReplanner::plan()  // //{
{
  const auto total_start = std::chrono::steady_clock::now();
  const auto elapsedMs = [](const std::chrono::steady_clock::time_point& start,
                            const std::chrono::steady_clock::time_point& stop) {
    return std::chrono::duration<double, std::milli>(stop - start).count();
  };

  const auto init_start = std::chrono::steady_clock::now();
  initializationPlan();
  const auto init_stop = std::chrono::steady_clock::now();

  auto start = std::chrono::steady_clock::now();
  fillAndInflateGrid(_inflated_grid_, cloud_);
  auto stop = std::chrono::steady_clock::now();
  const double inflate_ms = elapsedMs(start, stop);
  // std::cout << "[RBLReplanner]: Time taken by fillAndInflateGrid: " << duration_inflate.count() << " microseconds" <<
  // std::endl;

  const auto world_path_start = std::chrono::steady_clock::now();
  _path_ = worldPathToGridPath(path_);
  const auto world_path_stop = std::chrono::steady_clock::now();

  const auto should_replan_start = std::chrono::steady_clock::now();
  const bool should_replan = shouldReplan(path_, agent_pos_, _path_, _inflated_grid_);
  const auto should_replan_stop = std::chrono::steady_clock::now();
  if (!should_replan) {
    const auto total_stop = std::chrono::steady_clock::now();
    std::cout << "[RBLReplanner][timing] plan total_ms=" << elapsedMs(total_start, total_stop)
              << ", init_ms=" << elapsedMs(init_start, init_stop)
              << ", inflate_ms=" << inflate_ms
              << ", world_path_ms=" << elapsedMs(world_path_start, world_path_stop)
              << ", should_replan_ms=" << elapsedMs(should_replan_start, should_replan_stop)
              << ", clearance_ms=0"
              << ", astar_ms=0"
              << ", world_convert_ms=0"
              << ", replan=0"
              << ", cloud=" << (cloud_ ? cloud_->size() : 0)
              << ", grid=[" << _X_ << "," << _Y_ << "," << _Z_ << "]"
              << ", path=" << path_.size() << std::endl;
    return path_;
  }

  start = std::chrono::steady_clock::now();
  calculateClearanceGrid(_clearance_grid_, _inflated_grid_);
  stop = std::chrono::steady_clock::now();
  const double clearance_ms = elapsedMs(start, stop);
  // std::cout << "[RBLReplanner]: Time taken by calculateClearanceGrid: " << duration_clearance.count() << "
  // microseconds" << std::endl;

  start = std::chrono::steady_clock::now();
  _path_ = AStarPlan(_agent_pos_, _goal_, _path_, _inflated_grid_, _clearance_grid_);
  stop = std::chrono::steady_clock::now();
  const double astar_ms = elapsedMs(start, stop);

  start = std::chrono::steady_clock::now();
  path_ = gridPathToWorldPath(_path_);
  stop = std::chrono::steady_clock::now();
  const double world_convert_ms = elapsedMs(start, stop);
  // std::cout << "[RBLReplanner]: Time taken by AStarPlan: " << duration_a_star.count() << " microseconds" <<
  // std::endl; std::cout << "[RBLReplanner]: Overall planning took: " << (duration_inflate.count() +
  // duration_clearance.count() + duration_a_star.count()) / 1000 << " miliseconds" << std::endl;

  smooth_path_.clear();
  const auto total_stop = std::chrono::steady_clock::now();
  std::cout << "[RBLReplanner][timing] plan total_ms=" << elapsedMs(total_start, total_stop)
            << ", init_ms=" << elapsedMs(init_start, init_stop)
            << ", inflate_ms=" << inflate_ms
            << ", world_path_ms=" << elapsedMs(world_path_start, world_path_stop)
            << ", should_replan_ms=" << elapsedMs(should_replan_start, should_replan_stop)
            << ", clearance_ms=" << clearance_ms
            << ", astar_ms=" << astar_ms
            << ", world_convert_ms=" << world_convert_ms
            << ", replan=1"
            << ", cloud=" << (cloud_ ? cloud_->size() : 0)
            << ", grid=[" << _X_ << "," << _Y_ << "," << _Z_ << "]"
            << ", grid_path=" << _path_.size()
            << ", path=" << path_.size() << std::endl;
  return path_;
}  // //}

bool RBLReplanner::shouldReplan(const std::vector<Eigen::Vector3d>& path,
                                Eigen::Vector3d&                    agent_pos,
                                std::vector<std::tuple<int,
                                                       int,
                                                       int>>        _path,
                                std::optional<VoxelGrid>&           grid)  // //{
{
  if (goal_changed_) {
    std::cout << "[RBLReplanner]: Replanning, goal_changed " << std::endl;
    goal_changed_ = false;
    return true;
  }

  if (path.size() == 0) {
    // path is empty -> replan
    std::cout << "[RBLReplanner]: Replanning, because path lenght is 0. " << std::endl;
    return true;
  }

  // 50% completed? -> replan
  if (percentageCompleted(0.3, path, agent_pos)) {
    std::cout << "[RBLReplanner]: Replanning, because completed 50 percent of the path. " << std::endl;
    return true;
  }

  // path blocked? -> replan
  if (pathBlocked(_path, grid)) {
    std::cout << "[RBLReplanner]: Replanning, because path is blocked now. " << std::endl;
    return true;
  }

  return false;
}  // //}

bool RBLReplanner::percentageCompleted(const double                        percentage,
                                       const std::vector<Eigen::Vector3d>& path,
                                       Eigen::Vector3d&                    agent_pos)  // //{
// true -> will replan
// false will not replan based on this condition
{
  double min_dist_sq         = std::numeric_limits<double>::max();
  double path_length_sq      = 0.0;
  std::optional<size_t> closest_point_index;

  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      path_length_sq += (path[i] - path[i - 1]).squaredNorm();
    }
    double dist_sq = (path[i] - agent_pos).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq         = dist_sq;
      closest_point_index = i;
    }
  }

  if (!closest_point_index || *closest_point_index + 1 >= path.size()) {
    return true;
  }

  double length_to_closest_point_sq = 0.0;
  for (size_t i = 1; i < *closest_point_index; ++i) {
    length_to_closest_point_sq += (path[i] - path[i - 1]).squaredNorm();
  }

  if (length_to_closest_point_sq / path_length_sq >= percentage) {
    return true;
  }
  return false;
}  // //}

bool RBLReplanner::pathBlocked(std::vector<std::tuple<int,
                                                      int,
                                                      int>> _path,
                               std::optional<VoxelGrid>&    grid)  // //{
{
  if (!grid.has_value()) {
    return true;
  }

  int x, y, z;
  for (size_t i = 0; i < _path.size(); ++i) {
    x = std::get<0>(_path[i]);
    y = std::get<1>(_path[i]);
    z = std::get<2>(_path[i]);

    if (x < 0 || x >= grid->X || y < 0 || y >= grid->Y || z < 0 || z >= grid->Z) {
      return true;
    }

    if (grid->at(x, y, z) == 1) {
      return true;
    }
  }
  return false;
}  // //}

bool RBLReplanner::replanTimer()  // //{
{
  auto current_time = std::chrono::high_resolution_clock::now();
  if (first_plan) {
    first_plan  = false;
    last_replan = current_time;
    return true;
  }

  std::chrono::duration<double> diff = current_time - last_replan;

  if (diff.count() >= replanner_period_) {
    last_replan = current_time;
    return true;
  }
  return false;
}  // //}

void RBLReplanner::initializationPlan()  // //{
{
  _inflated_grid_->clear();
  _clearance_grid_->clear();
  _agent_pos_ =
      std::make_tuple(_X_ / 2, _Y_ / 2, std::max(static_cast<int>(altitude_ / params_.replanner_vox_size), 0));
  _goal_ = localSubgoalOnMapBoundary(_agent_pos_, worldCoordsToGridIdx(goal_));
}  // //}

// A*/RBL consensus: preserve goal direction and cap the local target by the reliable RBL radius.
std::tuple<int, int, int> RBLReplanner::localSubgoalOnMapBoundary(const std::tuple<int, int, int>& start,
                                                                  const std::tuple<int, int, int>& goal) const  // //{
{
  const int min_bounds[3] = {0, 0, 0};
  const int s[3] = {std::get<0>(start), std::get<1>(start), std::get<2>(start)};
  const int g[3] = {std::get<0>(goal), std::get<1>(goal), std::get<2>(goal)};
  int       max_bounds[3] = {_X_ - 1, _Y_ - 1, _Z_ - 1};

  if (std::isfinite(params_.max_flight_z)) {
    const int max_flight_z_idx =
        static_cast<int>(std::floor((params_.max_flight_z - agent_pos_.z()) / params_.replanner_vox_size)) + s[2];
    // A*/RBL altitude consensus: cap the local target by the safe controller altitude ceiling.
    max_bounds[2] = std::clamp(std::max(s[2], max_flight_z_idx), min_bounds[2], max_bounds[2]);
  }

  bool goal_inside = true;
  for (int axis = 0; axis < 3; ++axis) {
    if (g[axis] < min_bounds[axis] || g[axis] > max_bounds[axis]) {
      goal_inside = false;
      break;
    }
  }

  const double direction_x = static_cast<double>(g[0] - s[0]);
  const double direction_y = static_cast<double>(g[1] - s[1]);
  const double direction_z = static_cast<double>(g[2] - s[2]);
  const double distance_cells =
      std::sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z);

  if (distance_cells <= 1e-9) {
    return goal;
  }

  double t_exit = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    const int direction = g[axis] - s[axis];
    if (direction > 0) {
      t_exit = std::min(t_exit, static_cast<double>(max_bounds[axis] - s[axis]) / static_cast<double>(direction));
    }
    else if (direction < 0) {
      t_exit = std::min(t_exit, static_cast<double>(min_bounds[axis] - s[axis]) / static_cast<double>(direction));
    }
  }

  // A*/RBL consensus: do not commit the local A* target beyond the controller's reliable region.
  const double effective_radius_cells =
      params_.rbl_radius > 0.0 ? params_.rbl_radius / params_.replanner_vox_size : std::numeric_limits<double>::infinity();
  const double t_radius = effective_radius_cells / distance_cells;
  const double t_goal   = goal_inside ? 1.0 : std::numeric_limits<double>::infinity();
  double       t_subgoal = std::min({t_exit, t_radius, t_goal});

  if (!std::isfinite(t_exit)) {
    return std::make_tuple(std::clamp(g[0], min_bounds[0], max_bounds[0]),
                           std::clamp(g[1], min_bounds[1], max_bounds[1]),
                           std::clamp(g[2], min_bounds[2], max_bounds[2]));
  }

  if (!std::isfinite(t_subgoal)) {
    t_subgoal = t_exit;
  }

  const auto projectAxis = [&](const int axis) {
    const double projected = static_cast<double>(s[axis]) + t_subgoal * static_cast<double>(g[axis] - s[axis]);
    return std::clamp(static_cast<int>(std::round(projected)), min_bounds[axis], max_bounds[axis]);
  };

  return std::make_tuple(projectAxis(0), projectAxis(1), projectAxis(2));
}  // //}

double RBLReplanner::roundToNextMultiple(double value,
                                         double multiple)  // //{
{
  if (multiple == 0.0) {
    return value;
  }
  return std::ceil(value / multiple) * multiple;
}  // //}

Eigen::Vector3d RBLReplanner::gridIdxToWorldCoords(const std::tuple<int,
                                                                    int,
                                                                    int>& _point)  // //{
{
  Eigen::Vector3d point;
  point.x() = (std::get<0>(_point) - std::get<0>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.x();
  point.y() = (std::get<1>(_point) - std::get<1>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.y();
  point.z() = (std::get<2>(_point) - std::get<2>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.z();
  return point;
}  // //}

Eigen::Vector3d RBLReplanner::gridIdxToWorldCoords(const int& x,
                                                   const int& y,
                                                   const int& z)  // //{
{
  Eigen::Vector3d point;
  point.x() = (x - std::get<0>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.x();
  point.y() = (y - std::get<1>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.y();
  point.z() = (z - std::get<2>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.z();
  return point;
}  // //}

std::tuple<int,
           int,
           int>
RBLReplanner::worldCoordsToGridIdx(const Eigen::Vector3d& point)  // //{
{
  int x = static_cast<int>(std::round((point.x() - agent_pos_.x()) / params_.replanner_vox_size)) +
          std::get<0>(_agent_pos_);
  int y = static_cast<int>(std::round((point.y() - agent_pos_.y()) / params_.replanner_vox_size)) +
          std::get<1>(_agent_pos_);
  int z = static_cast<int>(std::round((point.z() - agent_pos_.z()) / params_.replanner_vox_size)) +
          std::get<2>(_agent_pos_);
  // std::cout << "[RBLReplanner] Returning x: " << x << ", y: " << y << ", z: " << z << std::endl;
  return std::make_tuple(x, y, z);
}  // //}

std::tuple<int,
           int,
           int>
RBLReplanner::worldCoordsToGridIdx(const pcl::PointXYZI& point)  // //{
{
  int x =
      static_cast<int>(std::round((point.x - agent_pos_.x()) / params_.replanner_vox_size)) + std::get<0>(_agent_pos_);
  int y =
      static_cast<int>(std::round((point.y - agent_pos_.y()) / params_.replanner_vox_size)) + std::get<1>(_agent_pos_);
  int z =
      static_cast<int>(std::round((point.z - agent_pos_.z()) / params_.replanner_vox_size)) + std::get<2>(_agent_pos_);
  // std::cout << "[RBLReplanner] Returning x: " << x << ", y: " << y << ", z: " << z << std::endl;
  return std::make_tuple(x, y, z);
}  // //}

void RBLReplanner::fillAndInflateGrid(std::optional<VoxelGrid>&                              grid,
                                      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  if (!grid.has_value() || !cloud) {
    return;
  }

  std::vector<std::tuple<int, int, int>> occupied;
  for (const auto& point : cloud->points) {
    auto _point = worldCoordsToGridIdx(point);
    const int x = std::get<0>(_point);
    const int y = std::get<1>(_point);
    const int z = std::get<2>(_point);

    if (x >= 0 && x < grid->X && y >= 0 && y < grid->Y && z >= 0 && z < grid->Z) {
      grid->at(x, y, z) = 1;
      occupied.push_back(_point);
    }
  }

  const int inflation_radius_sq = inflation_coeff_ * inflation_coeff_;
  for (const auto& idx : occupied) {
    const int x = std::get<0>(idx);
    const int y = std::get<1>(idx);
    const int z = std::get<2>(idx);

    for (int dx = -inflation_coeff_; dx <= inflation_coeff_; ++dx) {
      for (int dy = -inflation_coeff_; dy <= inflation_coeff_; ++dy) {
        for (int dz = -inflation_coeff_; dz <= inflation_coeff_; ++dz) {
          if (dx * dx + dy * dy + dz * dz > inflation_radius_sq) {
            continue;
          }

          const int nx = x + dx;
          const int ny = y + dy;
          const int nz = z + dz;
          if (nx >= 0 && nx < grid->X && ny >= 0 && ny < grid->Y && nz >= 0 && nz < grid->Z) {
            grid->at(nx, ny, nz) = 1;
          }
        }
      }
    }
  }

  for (int x = 0; x < grid->X; ++x) {
    for (int y = 0; y < grid->Y; ++y) {
      grid->at(x, y, 0) = 1;  // fill the floor
    }
  }
}  // //}

// Felzenszwalb & Huttenlocher distance transform, applied line-by-line.
void RBLReplanner::calculateClearanceGrid(std::optional<VoxelGrid>& clearance,
                                          const std::optional<VoxelGrid>& input)
{
  if (!input.has_value() || !clearance.has_value()) {
    return;
  }

  const int INF = 1e9;

  for (int x = 0; x < _X_; ++x) {
    for (int y = 0; y < _Y_; ++y) {
      for (int z = 0; z < _Z_; ++z) {
        clearance->at(x, y, z) = input->at(x, y, z) ? 0 : INF;
      }
    }
  }

  std::vector<int> f;
  std::vector<int> d;

  f.resize(_X_);
  d.resize(_X_);
  for (int y = 0; y < _Y_; ++y) {
    for (int z = 0; z < _Z_; ++z) {
      for (int x = 0; x < _X_; ++x) {
        f[x] = clearance->at(x, y, z);
      }
      calculate1dSquaredDistance(f.data(), d.data(), _X_);
      for (int x = 0; x < _X_; ++x) {
        clearance->at(x, y, z) = d[x];
      }
    }
  }

  f.resize(_Y_);
  d.resize(_Y_);
  for (int x = 0; x < _X_; ++x) {
    for (int z = 0; z < _Z_; ++z) {
      for (int y = 0; y < _Y_; ++y) {
        f[y] = clearance->at(x, y, z);
      }
      calculate1dSquaredDistance(f.data(), d.data(), _Y_);
      for (int y = 0; y < _Y_; ++y) {
        clearance->at(x, y, z) = d[y];
      }
    }
  }

  f.resize(_Z_);
  d.resize(_Z_);
  for (int x = 0; x < _X_; ++x) {
    for (int y = 0; y < _Y_; ++y) {
      for (int z = 0; z < _Z_; ++z) {
        f[z] = clearance->at(x, y, z);
      }
      calculate1dSquaredDistance(f.data(), d.data(), _Z_);
      for (int z = 0; z < _Z_; ++z) {
        clearance->at(x, y, z) = static_cast<int>(std::floor(std::sqrt(d[z])));
      }
    }
  }
}

void RBLReplanner::calculate1dSquaredDistance(int* f, int* d, int n)
{
  if (n <= 0) {
    return;
  }

  std::vector<int> v(n);
  std::vector<double> z(n + 1);

  int k = 0;
  v[0] = 0;
  z[0] = -1e20;
  z[1] = 1e20;

  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const int r = v[k];
      s = ((f[q] + q * q) - (f[r] + r * r)) / (2.0 * (q - r));
      if (s > z[k]) {
        break;
      }
      --k;
      if (k < 0) {
        s = -1e20;
        break;
      }
    }

    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e20;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const int r = v[k];
    d[q] = (q - r) * (q - r) + f[r];
  }
}

std::vector<Eigen::Vector3d> RBLReplanner::gridPathToWorldPath(std::vector<std::tuple<int,
                                                                                      int,
                                                                                      int>>& _path)  // //{
{
  std::vector<Eigen::Vector3d> path;
  if (_path.empty()) {
    return path;
  }
  for (auto _point : _path) {
    path.push_back(gridIdxToWorldCoords(_point));
  }
  return path;
}  // //}

std::vector<std::tuple<int,
                       int,
                       int>>
RBLReplanner::worldPathToGridPath(const std::vector<Eigen::Vector3d>& path)  // //{
{
  std::vector<std::tuple<int, int, int>> _path;
  for (const auto& point : path) {
    _path.push_back(worldCoordsToGridIdx(point));
  }
  return _path;
}  // //}

std::vector<std::tuple<int,
                       int,
                       int>>
RBLReplanner::smoothPath(const std::vector<std::tuple<int,
                                                      int,
                                                      int>>& _path,
                         const std::optional<VoxelGrid>&     grid,
                         const std::optional<VoxelGrid>*     clearance_grid)  // //{
{
  std::vector<std::tuple<int, int, int>> _smooth_path_fwrd;
  if (_path.empty()) {
    return _smooth_path_fwrd;
  }
  if (_path.size() <= 2) {
    return _path;
  }
  _smooth_path_fwrd.push_back(_path.front());

  // Forward pass
  size_t last_smooth_idx = 0;
  for (size_t i = 1; i < _path.size(); ++i) {
    if (canConnectPoints(_path[last_smooth_idx], _path[i], grid, clearance_grid)) {
      continue;
    }
    else {
      _smooth_path_fwrd.push_back(_path[i - 1]);
      last_smooth_idx = i - 1;
      i               = last_smooth_idx + 1;
    }
  }
  _smooth_path_fwrd.push_back(_path.back());

  // Backward pass
  std::vector<std::tuple<int, int, int>> final_smooth_path;
  final_smooth_path.push_back(_smooth_path_fwrd.back());
  size_t last_smooth_idx_bck = _smooth_path_fwrd.size() - 1;
  for (size_t i = _smooth_path_fwrd.size() - 1; i > 0; --i) {
    if (canConnectPoints(_smooth_path_fwrd[last_smooth_idx_bck], _smooth_path_fwrd[i], grid, clearance_grid)) {
      continue;
    }
    else {
      final_smooth_path.push_back(_smooth_path_fwrd[i + 1]);
      last_smooth_idx_bck = i + 1;
      i                   = last_smooth_idx_bck - 1;
    }
  }
  final_smooth_path.push_back(_smooth_path_fwrd.front());
  std::reverse(final_smooth_path.begin(), final_smooth_path.end());

  return final_smooth_path;
}  // //}

bool RBLReplanner::canConnectPoints(const std::tuple<int,
                                                     int,
                                                     int>&          p1,
                                    const std::tuple<int,
                                                     int,
                                                     int>&          p2,
                                    const std::optional<VoxelGrid>& grid,
                                    const std::optional<VoxelGrid>* clearance_grid)  // //{
{
  if (!grid.has_value()) {
    return false;
  }

  const auto inBounds = [&grid](const int x, const int y, const int z) {
    return x >= 0 && x < grid->X && y >= 0 && y < grid->Y && z >= 0 && z < grid->Z;
  };

  const bool use_clearance = clearance_grid != nullptr && clearance_grid->has_value();
  const int  minimum_clearance_cells =
      std::max(1, static_cast<int>(std::ceil(params_.encumbrance / params_.replanner_vox_size)));

  int x1 = std::get<0>(p1);
  int y1 = std::get<1>(p1);
  int z1 = std::get<2>(p1);

  int x2 = std::get<0>(p2);
  int y2 = std::get<1>(p2);
  int z2 = std::get<2>(p2);

  if (!inBounds(x1, y1, z1) || !inBounds(x2, y2, z2)) {
    return false;
  }

  double dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
  if (dist == 0.0) {
    if (grid.value().at(x1, y1, z1) != 0) {
      return false;
    }
    return !use_clearance || clearance_grid->value().at(x1, y1, z1) >= minimum_clearance_cells;
  }
  double step = 0.5;

  for (double t = 0; t <= dist; t += step) {
    double ratio = t / dist;
    int    x     = static_cast<int>(std::round(x1 + (x2 - x1) * ratio));
    int    y     = static_cast<int>(std::round(y1 + (y2 - y1) * ratio));
    int    z     = static_cast<int>(std::round(z1 + (z2 - z1) * ratio));

    if (!inBounds(x, y, z)) {
      return false;
    }

    if (grid.value().at(x, y, z) == 1) {
      return false;
    }

    if (use_clearance && clearance_grid->value().at(x, y, z) < minimum_clearance_cells) {
      return false;
    }
  }

  return true;
}  // //}

std::vector<std::tuple<int,
                       int,
                       int>>
RBLReplanner::AStarPlan(const std::tuple<int,
                                         int,
                                         int>               _start,
                        const std::tuple<int,
                                         int,
                                         int>               _goal,
                        const std::vector<std::tuple<int,
                                                     int,
                                                     int>>& _path,
                        const std::optional<VoxelGrid>&     grid,
                        const std::optional<VoxelGrid>&     clearance_grid)  // //{
{
  if (!grid.has_value() || !clearance_grid.has_value()) {
    return {};
  }

  Node* start_node = new Node(nullptr, closestFreeIdx(_start, grid));
  Node* end_node   = new Node(nullptr, closestFreeIdx(_goal, grid));

  std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
  VoxelGrid                                                   closed_voxels(_X_, _Y_, _Z_);
  std::vector<double>                                         best_g_score(grid->data.size(), std::numeric_limits<double>::infinity());
  const auto flatIndex = [&grid](const std::tuple<int, int, int>& position) {
    return std::get<0>(position) * grid->Y * grid->Z + std::get<1>(position) * grid->Z + std::get<2>(position);
  };

  best_g_score[flatIndex(start_node->position)] = 0.0;
  open_list.push(start_node);
  std::vector<Node*> all_allocated_nodes;
  all_allocated_nodes.push_back(start_node);
  all_allocated_nodes.push_back(end_node);

  size_t expanded_nodes     = 0;
  size_t generated_nodes    = 0;
  size_t skipped_oob        = 0;
  size_t skipped_occupied   = 0;
  size_t skipped_clearance  = 0;
  size_t skipped_altitude   = 0;
  size_t skipped_closed     = 0;
  size_t skipped_not_better = 0;
  size_t stale_open_entries = 0;
  size_t max_open_size      = open_list.size();

  while (!open_list.empty()) {
    Node* current_node = open_list.top();
    open_list.pop();
    if (closed_voxels.at(current_node->position)) {
      ++stale_open_entries;
      continue;
    }

    closed_voxels.at(current_node->position) = 1;
    ++expanded_nodes;

    if (*current_node == *end_node) {  // reconstruct the path
      // std::cout << "[RBLReplanner]: Reconstructing path." << std::endl;
      Node*                                  current = current_node;
      std::vector<std::tuple<int, int, int>> _path;
      // if (grid->at(_goal) == 1) {
      //   path_idx.push_back(_goal); // because at the start of the func I find the closest free idx on the grid
      // }
      while (current != nullptr) {  // nullptr means start
        _path.push_back(current->position);
        current = current->parent;
      }
      // if (grid->at(_start) == 1) {
      //   path_idx.push_back(_start); // because at the start of the func I find the closest free idx on the grid
      // }
      std::reverse(_path.begin(), _path.end());

      // std::vector<Eigen::Vector3d> path;
      // for (auto _point: _path) {
      //   path.push_back(gridIdxToWorldCoords(_point));
      // }

      for (Node* node : all_allocated_nodes)
        delete node;
      std::cout << "[RBLReplanner][astar] found path=" << _path.size()
                << ", expanded=" << expanded_nodes
                << ", generated=" << generated_nodes
                << ", skipped_oob=" << skipped_oob
                << ", skipped_occupied=" << skipped_occupied
                << ", skipped_clearance=" << skipped_clearance
                << ", skipped_altitude=" << skipped_altitude
                << ", skipped_closed=" << skipped_closed
                << ", skipped_not_better=" << skipped_not_better
                << ", stale_open=" << stale_open_entries
                << ", max_open=" << max_open_size << std::endl;
      return _path;
    }
    int                new_positions[][3] = { { 0, -1, 0 },   { 0, 1, 0 },   { -1, 0, 0 },  { 1, 0, 0 },
                                              { 0, 0, -1 },   { 0, 0, 1 },                                 // Face
                                              { -1, -1, 0 },  { -1, 1, 0 },  { 1, -1, 0 },  { 1, 1, 0 },   // Edge XY
                                              { 0, -1, -1 },  { 0, -1, 1 },  { 0, 1, -1 },  { 0, 1, 1 },   // Edge YZ
                                              { -1, 0, -1 },  { -1, 0, 1 },  { 1, 0, -1 },  { 1, 0, 1 },   // Edge XZ
                                              { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 },  // Corner
                                              { 1, -1, -1 },  { 1, -1, 1 },  { 1, 1, -1 },  { 1, 1, 1 } };

    for (int i = 0; i < 26; ++i) {
      std::tuple<int, int, int> node_position = { std::get<0>(current_node->position) + new_positions[i][0],
                                                  std::get<1>(current_node->position) + new_positions[i][1],
                                                  std::get<2>(current_node->position) + new_positions[i][2] };

      if (std::get<0>(node_position) > grid->X - 1 ||
          std::get<0>(node_position) < 0 ||  // check if outside of local map where mapping and planning is happening
          std::get<1>(node_position) > grid->Y - 1 || std::get<1>(node_position) < 0 ||
          std::get<2>(node_position) > grid->Z - 1 || std::get<2>(node_position) < 0) {
        ++skipped_oob;
        continue;
      }

      if (std::isfinite(params_.max_flight_z)) {
        const double node_z = (std::get<2>(node_position) - std::get<2>(_agent_pos_)) * params_.replanner_vox_size +
                              agent_pos_.z();
        // A*/RBL altitude consensus: never plan a waypoint above the safe controller ceiling.
        if (node_z > params_.max_flight_z + 1e-9) {
          ++skipped_altitude;
          continue;
        }
      }

      if (grid->at(std::get<0>(node_position), std::get<1>(node_position), std::get<2>(node_position)) !=
          0) {  // check if free space
        ++skipped_occupied;
        continue;
      }

      if (closed_voxels.at(node_position)) {
        ++skipped_closed;
        continue;
      }

      const double dist_parent_child = euclideanDistance(current_node->position, node_position);
      const double clearance         = params_.replanner_vox_size * clearance_grid->at(node_position);
      // A*/RBL consensus: reject cells that are too close to the already-inflated occupancy boundary.
      if (params_.min_clearance > 0.0 && clearance < params_.min_clearance) {
        ++skipped_clearance;
        continue;
      }

      double safety_penalty = params_.weight_safety / (clearance + params_.eps);
      double deviation_penalty =
          params_.weight_deviation * deviationPenalty(_path, current_node->position, node_position);
      const double virtual_obstacle_penalty = virtualObstaclePenalty(gridIdxToWorldCoords(node_position));
      // A*/RBL consensus: keep far-away nodes usable, but prefer the controller's reliable radius.
      const double dist_from_agent = euclideanDistance(_start, node_position);
      const double outside_rbl     = params_.rbl_radius > 0.0 ? std::max(0.0, dist_from_agent - params_.rbl_radius) : 0.0;
      const double rbl_penalty     = params_.outside_rbl_weight * outside_rbl * outside_rbl;
      const double tentative_g =
          current_node->g + dist_parent_child + safety_penalty + deviation_penalty + rbl_penalty + virtual_obstacle_penalty;
      const int    child_idx       = flatIndex(node_position);
      if (tentative_g + 1e-9 >= best_g_score[child_idx]) {
        ++skipped_not_better;
        continue;
      }

      best_g_score[child_idx] = tentative_g;

      Node* child = new Node(current_node, node_position);
      all_allocated_nodes.push_back(child);
      ++generated_nodes;
      child->g = tentative_g;
      child->h = euclideanDistance(child->position, end_node->position);
      child->f = child->g + child->h;

      open_list.push(child);
      max_open_size = std::max(max_open_size, open_list.size());
    }
  }

  for (Node* node : all_allocated_nodes)
    delete node;

  std::cout << "[RBLReplanner]: No path found. expanded=" << expanded_nodes
            << ", generated=" << generated_nodes
            << ", skipped_oob=" << skipped_oob
            << ", skipped_occupied=" << skipped_occupied
            << ", skipped_clearance=" << skipped_clearance
            << ", skipped_altitude=" << skipped_altitude
            << ", skipped_closed=" << skipped_closed
            << ", skipped_not_better=" << skipped_not_better
            << ", stale_open=" << stale_open_entries
            << ", max_open=" << max_open_size << std::endl;

  return {};
}  // //}

double RBLReplanner::deviationPenalty(const std::vector<std::tuple<int,
                                                                   int,
                                                                   int>>& _path,
                                      const std::tuple<int,
                                                       int,
                                                       int>&              _p1,
                                      const std::tuple<int,
                                                       int,
                                                       int>&              _p2)  // //{
{
  for (size_t i = 0; i < _path.size(); ++i) {
    if (_path[i] == _p1) {
      if (i + 1 < _path.size()) {
        if (_path[i + 1] == _p2) {
          return 0.0;
        }
      }
    }
  }
  return 1.0;
}  // //}

double RBLReplanner::virtualObstaclePenalty(const Eigen::Vector3d& point) const  // //{
{
  double penalty = 0.0;
  for (const auto& obstacle : virtual_obstacles_) {
    if (obstacle.radius <= 1e-6 || obstacle.weight <= 0.0) {
      continue;
    }

    const double distance = (point - obstacle.center).norm();
    if (distance >= obstacle.radius) {
      continue;
    }

    const double normalized = 1.0 - distance / obstacle.radius;
    // Recovery/stuck detection: quadratic soft penalty keeps the area usable if it is the only exit,
    // but makes A* prefer paths that do not re-enter a repeated stuck zone.
    penalty += obstacle.weight * normalized * normalized;
  }
  return penalty;
}  // //}

double RBLReplanner::euclideanDistance(const std::tuple<int,
                                                        int,
                                                        int>& p1,
                                       const std::tuple<int,
                                                        int,
                                                        int>& p2)  // //{
{
  return params_.replanner_vox_size *
         sqrt(pow(std::get<0>(p1) - std::get<0>(p2), 2) + pow(std::get<1>(p1) - std::get<1>(p2), 2) +
              pow(std::get<2>(p1) - std::get<2>(p2), 2));
}  // //}

std::tuple<int,
           int,
           int>
RBLReplanner::closestFreeIdx(const std::tuple<int,
                                              int,
                                              int>&          _position,
                             const std::optional<VoxelGrid>& grid)  // //{
{
  std::tuple<int, int, int> clamped_position = {
      std::clamp(std::get<0>(_position), 0, grid->X - 1),
      std::clamp(std::get<1>(_position), 0, grid->Y - 1),
      std::clamp(std::get<2>(_position), 0, grid->Z - 1),
  };

  if (grid->at(clamped_position) == 0) {
    return clamped_position;
  }
  std::queue<std::tuple<int, int, int>> q;
  std::set<std::tuple<int, int, int>>   visited;

  q.push(clamped_position);
  visited.insert(clamped_position);

  int new_positions[][3] = { { 0, -1, 0 },   { 0, 1, 0 },   { -1, 0, 0 },  { 1, 0, 0 },  { 0, 0, -1 },  { 0, 0, 1 },
                             { -1, -1, 0 },  { -1, 1, 0 },  { 1, -1, 0 },  { 1, 1, 0 },  { 0, -1, -1 }, { 0, -1, 1 },
                             { 0, 1, -1 },   { 0, 1, 1 },   { -1, 0, -1 }, { -1, 0, 1 }, { 1, 0, -1 },  { 1, 0, 1 },
                             { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 }, { 1, -1, -1 }, { 1, -1, 1 },
                             { 1, 1, -1 },   { 1, 1, 1 } };

  while (!q.empty()) {
    std::tuple<int, int, int> current_pos = q.front();
    q.pop();

    for (int i = 0; i < 26; ++i) {
      std::tuple<int, int, int> next_pos = { std::get<0>(current_pos) + new_positions[i][0],
                                             std::get<1>(current_pos) + new_positions[i][1],
                                             std::get<2>(current_pos) + new_positions[i][2] };

      if (visited.count(next_pos)) {
        continue;
      }

      if (std::get<0>(next_pos) < 0 || std::get<0>(next_pos) >= grid->X || std::get<1>(next_pos) < 0 ||
          std::get<1>(next_pos) >= grid->Y || std::get<2>(next_pos) < 0 || std::get<2>(next_pos) >= grid->Z) {
        continue;
      }

      if (grid->at(std::get<0>(next_pos), std::get<1>(next_pos), std::get<2>(next_pos)) == 0) {
        return next_pos;  // Found the closest free spot
      }

      visited.insert(next_pos);
      q.push(next_pos);
    }
  }
  return clamped_position;
}  // //}
