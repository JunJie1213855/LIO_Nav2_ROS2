// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_manage/warm_start_path.h>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace scan_planner
{
  namespace
  {
    // 按照 旅程路径/索引 对高度 z 进行线性插值
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      // 查看点云是否唯恐
      if (points.empty())
        return;

      // 点云数量为 1 时，直接取 start_z 为高度
      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }
      // xoy平面上的累积旅程距离 s
      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        // 计算累积距离 s
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }
      // 获取起点到终点的距离
      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length                       //  total_xy_length 大于 1e-6，则按照旅程距离线性插值 r = s[i] / s[N]
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1); //  total_xy_length 小于 1e-6，则按照索引距离线性插值 r = i / N
        // z 轴线性插值 z[i] = s_z + r * (t_z -  s_z)
        points[i](2) = start_z + ratio * (target_z - start_z);
      }
      // 起点和终点的高度设置
      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  // 模块初始化
  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    // 软约束
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);  // 速度
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);  // 加速度
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);// 加加速度
    // 忍受力
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    // 控制点距离
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    max_warm_start_deviation_ = get_double("manager.max_warm_start_deviation", 0.5);

    if (!std::isfinite(max_warm_start_deviation_) || max_warm_start_deviation_ <= 0.0)
      throw std::invalid_argument("manager.max_warm_start_deviation must be positive and finite");

    local_data_.traj_id_ = 0;

    // Grid Map 初始化
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    // B 样条优化其初始化
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);

    // 设置 A* 算法
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    
    // 初始化 Grid Map / Sliding Map 为 200 x 200 x 100 
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(200, 200, 100));

    // 可视化模块
    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;
    
    // 1.目标到达判定 < 0.2 m 就是到达了，直接返回
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    // 记录开始时间
    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0; // 初始化时间、优化时间、细化时间
 
    // 2.初始化 pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    // 时间补偿
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? 
                pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 :  // 较远的时候，因子小 1.2
                pp_.ctrl_pt_dist / pp_.max_vel_ * 5;     // 较近的时候，因子大 5
    // 初始路径点 point_set，
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      // 清除所有内容
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false; 

      // 是否为第一次初始化、明确要求重新生成多项式初值、强制使用多项式轨迹
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        // 复位标志
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        // 用梯形速度曲线估算总时间 time，加速 -> 匀速 -> 再减速
        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? 
                      sqrt(dist / pp_.max_acc_) : 
                      (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

       
        if (!flag_randomPolyTraj)  // 利用五次多项式生成初始化直线
        {
          // 轨迹生成
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else                      // 直接生成开始点到局部目标点的直线估计，插入随机中间点
        {
          // 生成起点到目标点的一个坐标轴
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized(); // 与 z 轴叉乘，得到水平面内、垂直于飞行方向的横向单位向量
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();             // 再与横向叉乘，得到垂直于前两者的方向，近似竖直方向。
          
          // 在起终点中点加随机偏移，偏移量随失败次数增大
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          // 中间点、随机点、终点，生成两段式路径
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2; // 按照 time 分成两段时间
          // 生成两段等时的 min-snap 轨迹
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        // 第一次不进行重采样
        ts *= 1.5;
        // 重采样，如果 1.距离大于 pp_.ctrl_pt_dist * 1.5 或者 2.点数小于 7，就将采样时间 ts 按照 1.5 倍频率重新采样
        do
        {
          ts /= 1.5;         // 重采样，第一次取时间不会重采样
          point_set.clear();
          flag_too_far = false;
          // 获取最终点
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          // [0, time] dt = ts
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            // 距离过大，重采样
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // 对三次 B 样条至少需要 7 个点，来初始化

        // 循环结束时 t 已越过 time，回退一步得到最后一个实际采样时刻
        t -= ts;
        // 下面四个量，会交给 B 样条，然后进行轨迹优化
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));  // 起点速度
        start_end_derivatives.push_back(local_target_vel);        // 终点速度（显式指定）
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));  // 起点加速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));  // 终点加速度
      }
      else // 非首次生成轨迹，走剩下的轨迹，planner 是局部规划器，局部由滑动窗口决定
      {
        // 获取从开始时间到现在已经到达的时间差
        const double t_cur = (node_->now() - local_data_.start_time_).seconds();
        // 查看是否为奇异值
        if (!std::isfinite(t_cur) || !std::isfinite(local_data_.duration_) ||
            t_cur < 0.0 || t_cur >= local_data_.duration_ - 1e-3 ||
            !std::isfinite(ts) || ts <= 0.0)
          return false;

        // 获取当前时刻期望到达的路径点 old_start
        const Eigen::Vector3d old_start = local_data_.position_traj_.evaluateDeBoorT(t_cur);
        // start_pt 为实际的规划起点，从里程计中获取，correction 则是期望点到实际点的偏移量
        const Eigen::Vector3d correction = start_pt - old_start;
        if (!old_start.allFinite() || !correction.allFinite() ||
            correction.norm() > max_warm_start_deviation_)
          return false;

        // 将实际起点作为新路径第一个点
        vector<Eigen::Vector3d> segment_point{start_pt};
        // 修正过渡时间，取 1s 和旧轨迹时间的最小值。修正过渡时间指的是将现在的点修正到旧轨迹上的点的时间
        const double blend_duration = std::min(1.0, local_data_.duration_ - t_cur);
        // 利用过渡时间，使得近处跟着机器人实际位置修正，远处尽量保留原来的绕障路线。
        for (double t = t_cur + ts; t < local_data_.duration_; t += ts)
        {
          // 修正权重 blend
          const double blend = std::max(0.0, 1.0 - (t - t_cur) / blend_duration);
          // 轨迹修正
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t) + blend * correction);
          // 点数保护，避免异常时长或过小采样间隔造成大量采样
          if (segment_point.size() > 10000)
            return false;
        }
        // 补上旧轨迹终点
        segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_));

        // 估计延伸段时间，乘 2 给运动留出时间，仍属于启发式估计
        const double poly_time = (segment_point.back() - local_target_pt).norm() / pp_.max_vel_ * 2;

        // 当前时间大于采样时间，则采用生成多项式延伸段
        if (poly_time > ts)
        {
          // 生成新终点的延伸段
          PolynomialTraj extension = PolynomialTraj::one_segment_traj_gen(
              segment_point.back(),
              local_data_.velocity_traj_.evaluateDeBoorT(local_data_.duration_),
              local_data_.acceleration_traj_.evaluateDeBoorT(local_data_.duration_),
              local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);
          // 继续采样
          for (double t = ts; t < poly_time; t += ts)
          {
            segment_point.push_back(extension.evaluate(t));
            if (segment_point.size() > 10000)
              return false;
          }
        }
        // 补上新目标
        segment_point.push_back(local_target_pt);

        // 重新采样，整理成 B-spline 参数化需要的点集
        const auto max_points = static_cast<std::size_t>(
            std::max(7.0, pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3.0));
        // 将拼接好的 segment_point 交给重采样函数，按给定空间间距和数量限制整理，输出到 point_set
        if (!resampleWarmStartPath(segment_point, pp_.ctrl_pt_dist, max_points, point_set))
          return false;
        // 记录雅可比式需要的数值
        start_end_derivatives.push_back(start_vel);              // 起始点速度
        start_end_derivatives.push_back(local_target_vel);       // 目标速度
        start_end_derivatives.push_back(start_acc);              // 起始点加速度
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());// 零向量
      }
    } while (flag_regenerate);

    // 3.z 轴插值，给每个路径点赋予 z 轴高度
    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    // 4.B 样条参数化，还没有开始优化
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 5.A*算法路径规划
    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true); // 内部包含碰撞查找、A*查找、路径优化

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // 可视化
    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    // 6.B样条轨迹优化，平滑、碰撞、物理速度以及加速度限制
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // 7.细化
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false)) // 检查可行性
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points); // 细化算法
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos)) // 动态可行性
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now()); // 更新轨迹信息

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
