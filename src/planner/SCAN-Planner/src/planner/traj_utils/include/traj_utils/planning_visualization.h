#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/uniform_bspline.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <traj_utils/polynomial_traj.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace scan_planner
{
  class PlanningVisualization
  {
  private:
    using MarkerPublisher = rclcpp::Publisher<visualization_msgs::msg::Marker>;
    using MarkerArrayPublisher = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>;
    rclcpp::Node *node_{nullptr};

    MarkerPublisher::SharedPtr goal_point_pub;
    MarkerPublisher::SharedPtr global_list_pub;
    MarkerPublisher::SharedPtr init_list_pub;
    MarkerPublisher::SharedPtr optimal_list_pub;
    MarkerPublisher::SharedPtr a_star_list_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    explicit PlanningVisualization(rclcpp::Node *node);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(const MarkerPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayOptimalTraj(UniformBspline position_traj, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(const MarkerArrayPublisher::SharedPtr &pub, const vector<Eigen::Vector3d> &list,
                          double scale, Eigen::Vector4d color, int id);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, scan_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, scan_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace scan_planner
#endif
