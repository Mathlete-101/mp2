#include "nav2_straightline_planner/straight_line_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "nav2_util/node_utils.hpp"
#include <cmath>

namespace nav2_straightline_planner
{

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  resolution_ = costmap_->getResolution();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  // Add parameter for cost threshold - set to a lower value to be more conservative
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_threshold", rclcpp::ParameterValue(20.0));
  node_->get_parameter(name_ + ".cost_threshold", cost_threshold_);

  // Add parameter for cost weight - higher value means more aggressive avoidance
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_weight", rclcpp::ParameterValue(2.0));
  node_->get_parameter(name_ + ".cost_weight", cost_weight_);
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLinePlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLinePlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

bool StraightLinePlanner::isValid(int x, int y)
{
  unsigned int ux = static_cast<unsigned int>(x);
  unsigned int uy = static_cast<unsigned int>(y);
  return x >= 0 && ux < costmap_->getSizeInCellsX() &&
         y >= 0 && uy < costmap_->getSizeInCellsY() &&
         costmap_->getCost(x, y) < cost_threshold_;
}

bool StraightLinePlanner::isGoal(const Node* current, const Node* goal)
{
  return current->x == goal->x && current->y == goal->y;
}

double StraightLinePlanner::calculateHeuristic(const Node* current, const Node* goal)
{
  // Using Euclidean distance as heuristic
  return std::hypot(goal->x - current->x, goal->y - current->y);
}

std::vector<Node*> StraightLinePlanner::getNeighbors(const Node* current)
{
  std::vector<Node*> neighbors;
  int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

  for (int i = 0; i < 8; ++i) {
    int new_x = current->x + dx[i];
    int new_y = current->y + dy[i];
    if (isValid(new_x, new_y)) {
      neighbors.push_back(new Node(new_x, new_y));
    }
  }
  return neighbors;
}

std::vector<Node*> StraightLinePlanner::reconstructPath(Node* goal)
{
  std::vector<Node*> path;
  Node* current = goal;
  while (current != nullptr) {
    path.push_back(current);
    current = current->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

void StraightLinePlanner::clearNodes(std::vector<Node*>& nodes)
{
  for (Node* node : nodes) {
    delete node;
  }
  nodes.clear();
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // Convert start and goal poses to costmap coordinates
  unsigned int start_x, start_y;
  unsigned int goal_x, goal_y;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal point is outside the costmap");
    return global_path;
  }

  // Create start and goal nodes
  Node* start_node = new Node(start_x, start_y);
  Node* goal_node = new Node(goal_x, goal_y);

  // Priority queue for open set
  std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;
  std::vector<Node*> all_nodes;  // To keep track of all nodes for cleanup

  // Add start node to open set
  start_node->g_cost = 0;
  start_node->h_cost = calculateHeuristic(start_node, goal_node);
  start_node->f_cost = start_node->g_cost + start_node->h_cost;
  open_set.push(start_node);
  all_nodes.push_back(start_node);

  // Hash map to store visited nodes and their costs
  std::unordered_map<int, double> visited;
  visited[start_node->x * costmap_->getSizeInCellsY() + start_node->y] = start_node->g_cost;

  bool found_path = false;
  Node* current = nullptr;

  while (!open_set.empty()) {
    current = open_set.top();
    open_set.pop();

    if (isGoal(current, goal_node)) {
      found_path = true;
      break;
    }

    std::vector<Node*> neighbors = getNeighbors(current);
    for (Node* neighbor : neighbors) {
      // Calculate base movement cost
      double movement_cost = std::hypot(neighbor->x - current->x, neighbor->y - current->y);
      
      // Add cost-based penalty - higher cost cells are more expensive to traverse
      double cell_cost = static_cast<double>(costmap_->getCost(neighbor->x, neighbor->y));
      double cost_penalty = (cell_cost / cost_threshold_) * cost_weight_;
      
      // Total cost is movement cost plus cost penalty
      double tentative_g_cost = current->g_cost + movement_cost * (1.0 + cost_penalty);

      int key = neighbor->x * costmap_->getSizeInCellsY() + neighbor->y;
      if (visited.find(key) == visited.end() || tentative_g_cost < visited[key]) {
        neighbor->parent = current;
        neighbor->g_cost = tentative_g_cost;
        neighbor->h_cost = calculateHeuristic(neighbor, goal_node);
        neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
        visited[key] = tentative_g_cost;
        open_set.push(neighbor);
        all_nodes.push_back(neighbor);
      } else {
        delete neighbor;
      }
    }
  }

  if (found_path) {
    std::vector<Node*> path = reconstructPath(current);
    
    // Convert path to poses
    for (Node* node : path) {
      geometry_msgs::msg::PoseStamped pose;
      double wx, wy;
      costmap_->mapToWorld(node->x, node->y, wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = start.pose.orientation;  // Keep the same orientation
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "No path found to goal");
  }

  // Clean up all allocated nodes
  clearNodes(all_nodes);
  delete goal_node;

  return global_path;
}

}  // namespace nav2_straightline_planner

// Register the plugin
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLinePlanner, nav2_core::GlobalPlanner) 