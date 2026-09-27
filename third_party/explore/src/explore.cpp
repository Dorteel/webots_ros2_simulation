/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , last_markers_count_(0)
{
  initial_scan_ = declare_parameter<bool>("initial_scan", false);
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  progress_distance_ = declare_parameter<double>("progress_distance", 0.15);
  navigation_timeout_ = declare_parameter<double>("navigation_timeout", 0.0);
  min_goal_distance_ = declare_parameter<double>("min_goal_distance", 0.6);
  goal_clearance_ = declare_parameter<double>("goal_clearance", 0.35);
  blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.5);
  frontier_standoff_ = declare_parameter<double>("frontier_standoff", 0.4);
  if (!(progress_timeout_ > 0 && progress_distance_ > 0 && navigation_timeout_ >= 0 &&
        min_goal_distance_ > 0 && goal_clearance_ > 0 && blacklist_radius_ > 0 &&
        frontier_standoff_ > 0 && planner_frequency_ > 0))
    throw std::invalid_argument("Exploration distances/timeouts/frequency must be positive (total timeout may be zero)");
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, "spin");
  planner_client_ = rclcpp_action::create_client<Planner>(this, "compute_path_to_pose");

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  completion_pub_ = create_publisher<std_msgs::msg::Bool>(
      "exploration_complete", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Bool incomplete;
  incomplete.data = false;
  completion_pub_->publish(incomplete);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  RCLCPP_INFO(logger_, "Exploration started.");
  makePlan();
}

Explore::~Explore()
{
  if (rclcpp::ok()) stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  if (stopped_ || planning_ || scanning_) return;
  if (initial_scan_) {
    if (!spin_client_->action_server_is_ready()) return;
    scanning_ = true;
    RCLCPP_INFO(logger_, "SLAM started; scanning initial blind area with a Nav2 half-turn");
    nav2_msgs::action::Spin::Goal goal;
    goal.target_yaw = 3.14159;
    goal.time_allowance.sec = 45;
    auto options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
    options.goal_response_callback = [this](auto handle) {
      if (!handle) {
        scanning_ = false;
        RCLCPP_ERROR(logger_, "Initial scan rejected; exploration paused. Resume when Nav2 is active.");
        stop();
      }
    };
    options.result_callback = [this](const auto& result) {
      scanning_ = false;
      if (stopped_) return;
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(logger_, "Initial scan failed; exploration paused. Check obstacles before resuming.");
        stop();
        return;
      }
      initial_scan_ = false;
      scan_ready_ = now() + rclcpp::Duration::from_seconds(4.0);
      RCLCPP_INFO(logger_, "Initial scan complete; waiting for SLAM map update");
    };
    spin_client_->async_send_goal(goal, options);
    return;
  }
  if (now() < scan_ready_) return;
  if (goal_active_) {
    if (!navigation_goal_handle_ || cancel_pending_) return;
    geometry_msgs::msg::Point robot;
    const auto stamp = now();
    if (robotPosition(robot)) {
      const double distance = std::hypot(robot.x - navigation_target_.x, robot.y - navigation_target_.y);
      const double remaining = (stamp - feedback_time_).seconds() < 5.0 ? remaining_ :
          std::numeric_limits<double>::infinity();
      if (progress_.update(stamp.seconds(), robot.x, robot.y, distance, remaining, progress_distance_) &&
          (stamp - progress_log_time_).seconds() >= 10.0) {
        RCLCPP_INFO(logger_, "Navigation progress: %.2f m to goal, elapsed %.1f s",
                    distance, stamp.seconds() - progress_.started);
        progress_log_time_ = stamp;
      }
    }
    const bool stalled = progress_.stalled(stamp.seconds(), progress_timeout_);
    if (stalled || progress_.expired(stamp.seconds(), navigation_timeout_)) {
      RCLCPP_WARN(logger_, "Navigation %s: no progress for %.1f s, elapsed %.1f s; canceling",
                  stalled ? "stalled" : "total timeout", stamp.seconds() - progress_.last_progress,
                  stamp.seconds() - progress_.started);
      blacklist(prev_goal_, stalled ? "no meaningful progress" : "total navigation timeout");
      cancel_pending_ = true;
      move_base_client_->async_cancel_goal(navigation_goal_handle_);
      // Keep ownership until the terminal result; never overlap navigation goals.
    }
    return;
  }
  if (!planner_client_->action_server_is_ready()) return;
  if (!tf_buffer_.canTransform(costmap_client_.getGlobalFrameID(), robot_base_frame_,
                               tf2::TimePointZero)) return;
  // find frontiers
  geometry_msgs::msg::Point robot;
  if (!robotPosition(robot)) return;
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(robot);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    if (++empty_checks_ < 5) return;
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  geometry_msgs::msg::PoseStamped candidate;
  auto frontier = std::find_if(frontiers.begin(), frontiers.end(),
      [this, &robot, &candidate](const auto& f) { return selectEndpoint(f, robot, candidate); });
  if (frontier == frontiers.end()) {
    // Let the SLAM update after reaching a goal before declaring exhaustion.
    if (++empty_checks_ < 5) return;
    RCLCPP_INFO(logger_, "No useful frontier endpoints remaining; Exploration complete");
    auto status = explore_lite_msgs::msg::ExploreStatus();
    status.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status);
    stop(true);
    return;
  }
  empty_checks_ = 0;
  const auto target_position = candidate.pose.position;
  prev_goal_ = target_position;
  RCLCPP_INFO(logger_, "Frontier selected: free endpoint %.2f, %.2f; checking Nav2 reachability",
              target_position.x, target_position.y);
  Planner::Goal request;
  request.goal = candidate;
  request.planner_id = "GridBased";
  request.use_start = false;
  planning_ = true;
  auto options = rclcpp_action::Client<Planner>::SendGoalOptions();
  options.goal_response_callback = [this, target_position](auto handle) {
    if (!handle) {
      planning_ = false;
      blacklist(target_position, "planner rejected candidate");
    }
  };
  options.result_callback = [this, target_position, candidate](const auto& result) {
    planning_ = false;
    if (stopped_) return;
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !result.result || result.result->path.poses.empty()) {
      blacklist(target_position, "unreachable candidate");
      return;
    }
    // Navfn tolerance may end in nearby free space. Navigate to that checked endpoint.
    auto endpoint = result.result->path.poses.back();
    geometry_msgs::msg::Point robot;
    if (!robotPosition(robot)) return;
    if (!validEndpoint(endpoint.pose.position) ||
        std::hypot(endpoint.pose.position.x - robot.x, endpoint.pose.position.y - robot.y) < min_goal_distance_ ||
        goalOnBlacklist(endpoint.pose.position)) {
      blacklist(target_position, "planner endpoint too close, blocked, or already tried");
      return;
    }
    // Face the unknown boundary rather than forcing every goal to world yaw zero.
    endpoint.pose.orientation = candidate.pose.orientation;
    sendNavigation(endpoint, target_position);
  };
  planner_client_->async_send_goal(request, options);
}

void Explore::sendNavigation(const geometry_msgs::msg::PoseStamped& pose,
                             const geometry_msgs::msg::Point& target_position)
{
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = pose;
  goal.pose.header.stamp = now();
  geometry_msgs::msg::Point robot;
  if (!robotPosition(robot)) return;
  navigation_target_ = pose.pose.position;
  progress_.reset(now().seconds(), robot.x, robot.y,
                  std::hypot(robot.x - navigation_target_.x, robot.y - navigation_target_.y));
  progress_log_time_ = now();
  remaining_ = std::numeric_limits<double>::infinity();
  navigation_goal_handle_.reset();
  cancel_pending_ = false;
  RCLCPP_INFO(logger_, "Navigation goal sent: %.2f, %.2f", pose.pose.position.x,
              pose.pose.position.y);
  goal_active_ = true;
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this, target_position](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
          blacklist(target_position, "navigation rejected");
        } else {
          navigation_goal_handle_ = goal_handle;
          active_goal_id_ = goal_handle->get_goal_id();
          if (stopped_) move_base_client_->async_cancel_goal(goal_handle);
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.feedback_callback = [this](auto handle, const auto feedback) {
    if (!goal_active_ || handle->get_goal_id() != active_goal_id_) return;
    // Ignore Nav2's uninitialized zero distance before it has a path.
    if (std::isfinite(feedback->distance_remaining) && feedback->distance_remaining > 0.0) {
      remaining_ = feedback->distance_remaining;
      feedback_time_ = now();
    }
  };

  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  for (const auto& p : frontier_blacklist_)
    if (std::hypot(goal.x - p.x, goal.y - p.y) < blacklist_radius_) return true;
  for (const auto& p : visited_goals_)
    if (std::hypot(goal.x - p.x, goal.y - p.y) < blacklist_radius_) return true;
  return false;
}

void Explore::blacklist(const geometry_msgs::msg::Point& point, const char* reason)
{
  if (!goalOnBlacklist(point)) frontier_blacklist_.push_back(point);
  RCLCPP_WARN(logger_, "Frontier blacklisted: %.2f, %.2f (%s)", point.x, point.y, reason);
}

bool Explore::robotPosition(geometry_msgs::msg::Point& point)
{
  try {
    const auto transform = tf_buffer_.lookupTransform(costmap_client_.getGlobalFrameID(),
                                                      robot_base_frame_, tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() != 0 && (now() - stamp).seconds() > 2.0) return false;
    point.x = transform.transform.translation.x;
    point.y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException&) { return false; }
}

bool Explore::validEndpoint(const geometry_msgs::msg::Point& point)
{
  auto* map = costmap_client_.getCostmap();
  unsigned int x, y;
  if (!map->worldToMap(point.x, point.y, x, y) || map->getCost(x, y) != 0) return false;
  const int radius = std::ceil(goal_clearance_ / map->getResolution());
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (std::hypot(dx, dy) * map->getResolution() > goal_clearance_) continue;
      const int xx = int(x) + dx, yy = int(y) + dy;
      if (xx < 0 || yy < 0 || xx >= int(map->getSizeInCellsX()) || yy >= int(map->getSizeInCellsY()))
        continue;
      const auto cost = map->getCost(xx, yy);
      if (cost != nav2_costmap_2d::NO_INFORMATION && cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        return false;
    }
  }
  return true;
}

bool Explore::selectEndpoint(const frontier_exploration::Frontier& frontier,
                            const geometry_msgs::msg::Point& robot,
                            geometry_msgs::msg::PoseStamped& goal)
{
  double best = std::numeric_limits<double>::infinity();
  auto* map = costmap_client_.getCostmap();
  const size_t stride = std::max(size_t(1), frontier.points.size() / 80);
  for (size_t i = 0; i < frontier.points.size(); i += stride) {
    const auto& edge = frontier.points[i];
    for (int direction = 0; direction < 16; ++direction) {
      const double angle = direction * 2.0 * M_PI / 16;
      geometry_msgs::msg::Point point;
      point.x = edge.x + frontier_standoff_ * std::cos(angle);
      point.y = edge.y + frontier_standoff_ * std::sin(angle);
      unsigned int x, y;
      if (!map->worldToMap(point.x, point.y, x, y)) continue;
      map->mapToWorld(x, y, point.x, point.y);
      const double distance = std::hypot(point.x - robot.x, point.y - robot.y);
      if (distance < min_goal_distance_ || distance >= best ||
          goalOnBlacklist(point) || !validEndpoint(point)) continue;
      best = distance;
      goal.header.frame_id = costmap_client_.getGlobalFrameID();
      goal.header.stamp = now();
      goal.pose.position = point;
      const double yaw = std::atan2(edge.y - point.y, edge.x - point.x);
      goal.pose.orientation.z = std::sin(yaw / 2);
      goal.pose.orientation.w = std::cos(yaw / 2);
    }
  }
  return std::isfinite(best);
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_ = false;
  navigation_goal_handle_.reset();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Navigation goal succeeded");
      visited_goals_.push_back(navigation_target_);
      scan_ready_ = now() + rclcpp::Duration::from_seconds(3.0);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(logger_, "Navigation goal failed");
      blacklist(frontier_goal, "Nav2 aborted navigation");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger_, "Navigation goal canceled%s", cancel_pending_ ? " after stall/timeout" : "");
      if (!stopped_ && !cancel_pending_) blacklist(frontier_goal, "Nav2 canceled navigation");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  stopped_ = true;
  if (finished_exploring && !completion_published_) {
    RCLCPP_INFO(logger_, "No useful frontiers remaining; exploration complete");
    completion_published_ = true;
    std_msgs::msg::Bool complete;
    complete.data = true;
    completion_pub_->publish(complete);
  }
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  if (scanning_) spin_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  stopped_ = false;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
