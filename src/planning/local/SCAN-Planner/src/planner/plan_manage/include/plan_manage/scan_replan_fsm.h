#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/execution_progress.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    // 状态机的状态枚举值
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    // 三种导航模式
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,  // 手动 2D 航点
      PRESET_TARGET = 2,  // 预设 3D 航点导航，可以由配置文件提供
      REFERENCE_PATH = 3, // 参考路径导航，可以由全局路径提供
    };

    // 工具类
    SCANPlannerManager::Ptr planner_manager_;    // 规划器管理器
    PlanningVisualization::Ptr visualization_;   // 可视化
    scan_planner_msgs::msg::DataDisp data_disp_; // 数据视差

    // 参数
    int navi_mode_; // 1 manual select, 2 hard code
    bool use_controller_progress_{false};
    int64_t controller_progress_id_{-1};
    double controller_progress_time_{0.0};
    rclcpp::Time controller_feedback_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Subscription<scan_planner_msgs::msg::ExecutionProgress>::SharedPtr controller_progress_sub_;
    void controllerProgressCallback(const scan_planner_msgs::msg::ExecutionProgress::ConstSharedPtr &msg);
    bool prefer_previous_trajectory_{true};
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    // 规划数据
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;

    // 里程计状态
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // 里程计状态，包括位置、速度和加速度
    Eigen::Quaterniond odom_orient_;                 // 里程计朝向，四元数

    // 导航状态
    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state，开始状态，包括初始化位置、开始位置、开始速度、开始加速度、开始角度
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state，目标状态，包括位置、速度
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state，局部目标状态，包括位置，速度
    std::vector<Eigen::Vector3d> active_waypoints_;                          // 
    int current_wp_;

    bool flag_escape_emergency_;

    // ROS 工具，节点、订阅者、发布者
    rclcpp::Node *node_{nullptr};  // 节点
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;                           // 定时器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;        // 目标订阅器
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                // 里程计订阅器
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;                    // 路径订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;    // go2 执行器
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;        // B样条发布器
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;     // 数据差距发布器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_; // 自我膨胀发布其

    // 帮助函数
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

    // ROS 回调函数
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
