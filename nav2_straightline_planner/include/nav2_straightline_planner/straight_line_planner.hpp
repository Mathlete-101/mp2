#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <functional>

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_straightline_planner
{

// Structure to represent a node in the A* search
struct Node
{
  int x, y;
  double g_cost;  // Cost from start to current
  double h_cost;  // Heuristic cost from current to goal
  double f_cost;  // Total cost (g_cost + h_cost)
  Node* parent;

  Node(int x_, int y_) : x(x_), y(y_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
};

// Custom comparator for priority queue
struct CompareNodes
{
  bool operator()(const Node* a, const Node* b)
  {
    return a->f_cost > b->f_cost;
  }
};

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Resolution of the costmap
  double resolution_;

  // Interpolation resolution for the path
  double interpolation_resolution_;

  // Cost threshold for valid cells
  double cost_threshold_;

  // Weight for cost-based path cost calculation
  double cost_weight_;

private:
  // Helper methods for A* search
  std::vector<Node*> getNeighbors(const Node* current);
  double calculateHeuristic(const Node* current, const Node* goal);
  bool isValid(int x, int y);
  bool isGoal(const Node* current, const Node* goal);
  std::vector<Node*> reconstructPath(Node* goal);
  void clearNodes(std::vector<Node*>& nodes);
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_ 