## 20210705

1. 学习程序：从MissionHandler到Thetastar是怎么调用的？

   * MissionBase基类定义了初始化、参考轨迹、规划、控制、获取目标、获取状态、暂停、恢复等虚函数，分电梯，充电桩，通道，平层导航等情况单独写成MissionBase的子类并重载planProcess方法。
   * 枚举类MissionMode定义了NormalNav,Gate,Recharge,Tracking...等。
   * MissionHandle私有成员：map<MissionMode, std::unique_ptr<MissionBase>> m_missions和atomic<MissionMode> m_mission_mode。
   *  MissionHandle中makePlan函数中通过用m_mission_mode在map（m_missions）中找出对应的unique_ptr再调用其planProcess方法。m_missions.at(m_mission_mode)->planProcess(m_cur_pose, speed,m_static_map,aux_data);
   * 以NormalNav为例，它的planProcess方法里根据switch (m_normal_nav_status)进行了startAlignTask，normalTask，moveTask,rotateTask,terminusAlignTask,ALOGE,ALOGD。其中startAlignTask中调用了thetastar:m_indoor_planner->startThetaStar();
2. 阅读论文：The Dynamic Window Approach to Collision Avoidance 
   * 搜索空间
     * 圆弧轨迹：考虑一系列圆弧轨迹组合，定义二维的速度搜索空间。
     * 允许速度：考虑一个制动过程，按照制动减速度计算到距离最近的障碍物时可以停车计算得到的制动初速度为对应弧线的允许速度。$V_a=\left\{(v,\omega)|v{\leq}\sqrt{2{\cdot}dist(v,\omega){\cdot}\dot{v_b}}\land\omega{\leq}\sqrt{2{\cdot}dist(v,\omega){\cdot}\dot{\omega_b}}\right\}$
     * 动态窗口：在一个很短的时间段内可以到达的速度和角速度$\{[v-{\Delta}v,v+{\Delta}v],[\omega-{\Delta}\omega,\omega-{\Delta}\omega]\}$，体现了有界的加速度作用。$V_d=\left\{(v,\omega)|v\in[v_a-\dot{v}{\cdot}t,v_a+\dot{v}{\cdot}t]\land\omega\in[\omega_a-\dot{\omega}{\cdot}t,\omega_a+\dot{\omega}{\cdot}t]\right\}$
   * 优化:$G()=\sigma({\alpha}{\cdot}heading(v,\omega)+\beta{\cdot}dist(v,\omega)+\gamma{\cdot}velocity(v,\omega))$最大
     * 目标航向：定义为$180-\theta$，$\theta$是预测状态的机器人朝向于目标方向之间的夹角，当机器人径直朝向目标时为0。
     * 距离障碍物距离：轨迹到最近障碍物的距离。
     * 速度。

## 20210706

1.学习程序DwaPlanner

* 允许速度的实现：遍历轨迹，将不符合速度约束的跳过别的进入一新容器。

  ```
  for (int i = 0; i < sample_trajs.size(); i++) {
          Eigen::Vector2f sample_speed = sample_trajs[i].wayPoint(0).vel;
          if (sample_speed[0] < v_min || sample_speed[0] > v_max
              || sample_speed[1] < w_min || sample_speed[1] > w_max) {
              continue;
          }
          useful_traj.emplace_back(sample_trajs[i]);
          collision_zones.emplace_back(trajs_collison_zone[i]);
      }
  ```

* 计算total cost:

  ```
  total_cost =cost_weight[0] * traj_dist_cost +cost_weight[1] * collision_cost +cost_weight[2] * 			            oscillation_cost +cost_weight[3] * orientation_cost;
  ```

  

* 寻找cost最大的轨迹：

  ```
    double cost_temp = 1e8;
      int id_temp = -1;
  
      for (int i = 0; i < useful_trajs.size(); i++){
          bool is_cost_valid = collision_cost_raw[i] < COLLISION_COST_MAX - 0.01f;
          bool is_orientation_valid = orientation_cost_raw[i] < m_max_orientation_offset;
          bool is_distance_valid = distance_terminal_to_reftraj[i] < m_max_dist_offset;
          if (cost_temp > total_cost[i] && is_cost_valid && is_orientation_valid && is_distance_valid) {
              cost_temp = total_cost[i];
              id_temp = i;
          }
  return id_temp;
  ```


2. 阅读论文：Trajectory modification considering dynamic constraints of autonomous robots
   * TEB将一系列路径点组成的初始路径转化为显示依赖时间变量的轨迹从而实时的控制机器人，解决环境动态变化的局部规划问题。
   * Timed Elastic Band:位姿$Q=\{x_i\}_{i=0...n}$，时间$\tau=\{{\Delta}T_i\}_{i=0...n-1}$，TEB定义序列同时含有位姿和时间$B:=(Q,\tau)$，然后优化：$minf(B)=min\{\sum_{k}\gamma_kf_k(B)\}$
     * TEB cost函数：分段连续，可微$e_\Gamma(x,x_r,\epsilon,S,n){\simeq}\left\{\begin{aligned}\left(\frac{x-(x_r-\epsilon)}{S}\right)^n&&if{\quad}x>x_r-\epsilon\\0&&{otherwise}\end{aligned}\right\}$
   * 路径点和障碍物:路径点使橡皮筋合并吸引，障碍物使橡皮筋排斥。$f_{path}=e_\Gamma(d_{min,j},r_{p_{max}},\epsilon,S,n)$，$f_{ob}=e_\Gamma(-d_{min,j},-r_{p_{max}},\epsilon,S,n)$，其梯度可以解释为作用在橡皮筋上的外力。
   * 速度和加速度:$v_i\simeq\frac{1}{{\Delta}T_i}\parallel\left(\begin{array}{1}x_{i+1}-x_i\\y_{i+1}-y_i\end{array}\right)\parallel$，$\omega_i\simeq\frac{\beta_{i+1}-\beta{i}}{{\Delta}T_i}$，$a_i=\frac{2（v_{i+1}-v_i）}{{\Delta}T_i+{\Delta}T_{i+1}}$
   * 非完整运动学：$f_k(x_i,x_{i+1})=\parallel{\left[\left(\begin{matrix}\cos\beta_i\\\sin\beta_i\\0\end{matrix}\right)+\left(\begin{matrix}\cos\beta_{i+1}\\\sin\beta_{i+1}\\0\end{matrix}\right)\right]\times\left(\begin{matrix}x_{i+1}-x_i\\y_{i+1}-y_i\\0\end{matrix}\right)}\parallel^2$
   * 最快路径：

## 20210707

1. CCBS算法的可视化问题：现在输出为XML格式
   * 找出现在程序中输出的格式规范
   * QT工具ASearchVisualizer可以可视化自己example文件夹中的xml。
2. g2o:general graph optimization
   * 选择一个线性方程求解器：从PCG，CSparse，Choldmod中选
   * 选择一个BlockSolver
   * 选择一个迭代测量，从Gauss-Newton,Levernberg-Marquardt,Powell's dogleg中选。

##  20210708 

1. QT：ASearchVisualizer程序：输入xml文件，鼠标滑动进度条展示寻路算法的状态。进度条头为算法的起点，进度条尾为算法的重点。

   * 程序是通过集成tinyxml开源C++的xml解析程序和QT的图窗，进度条等控件实现可视化。

2. Python:libMultiRobotPlanning程序：输入两个yaml文件，map_obstacke和 algorighm output输出为matplotlib库做的animation。

   * yaml库：解析yaml文件
   * matplotlib库：画图
   * argparse库：解析命令行输入
   * 使用方式是在命令行输入：python3 ../example/visualize.py ../benchmark/32x32_obst204/map_32by32_obst204_agents10_ex1.yaml output.yaml

   ##  20210709

