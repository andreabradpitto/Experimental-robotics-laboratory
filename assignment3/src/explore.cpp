#include <explore.h>
#include <std_msgs/Int64.h>

#include <thread>

#include <ros/ros.h>
#include <assignment3/Explore.h>

/** 
 * @brief This is the piece of code implementing the exploration algorithm. I added a pair
 * of methods inside the class in order to provide a start & stop service server feature
 * to the original code. These servers are used by dog_fsm.py and dog_vision.py
 */

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
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
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
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
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop();
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    stop();
    return;
  }
  geometry_msgs::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we have made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we do not need to do anything if we are still pursuing the same goal
  if (same_goal) {
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we are pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  frontier_blacklist_.clear();
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped");
}

/** Exploration algorithm start service server
 */
bool Explore::srv_start(assignment3::Explore::Request& req,
                 assignment3::Explore::Response& res)
{
  res.output = req.input;
  first_run = 0;
  start();
  return true;
}

/** Exploration algorithm stop service server
 */
bool Explore::srv_stop(assignment3::Explore::Request& req,
                 assignment3::Explore::Response& res)
{
  res.output = req.input;
  stop();
  return true;
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore_server_node");

  // Instantiating an object of the Explore class
  explore::Explore explore;

  if (explore.first_run == 1)
  {
    explore.stop();
  }

  /** Creation of the service servers called by dog_fsm.py and dog_vision.py as
   * methods of the Explore class
   */
  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::ServiceServer startService =
      n1.advertiseService("explore_start_service", &explore::Explore::srv_start, &explore);
  ros::ServiceServer stopService =
      n2.advertiseService("explore_stop_service", &explore::Explore::srv_stop, &explore);

  ros::spin();
  return 0;
}
