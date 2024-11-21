#include "mpc_ros1_rbt1.hpp"


double pi_to_pi(double angle)
{
    while(angle >= M_PI)
        angle -= 2.0 * M_PI;

    while(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

double q_to_e(nav_msgs::Odometry msg)
{
    tf::Quaternion q;
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    q.setW(msg.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

tf::Quaternion e_to_q(double roll, double pitch, double yaw)
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    // q.normalize(); // Uncomment if normalization is needed
    return q;
}
void mpc_ros1_rbt1::setting_reference()
{
    for(int n = 0; n < N; ++n)
    {
        double roll, pitch;
        double theta_ref = 0.0;
        double u_ref = v_max / (N * dt); //0.2
        double omega_ref = 0.0;

        tf::Quaternion q;
        q.setX(rbt_ref_traj_msg.poses[n].pose.orientation.x);
        q.setY(rbt_ref_traj_msg.poses[n].pose.orientation.y);
        q.setZ(rbt_ref_traj_msg.poses[n].pose.orientation.z);
        q.setW(rbt_ref_traj_msg.poses[n].pose.orientation.w);

        tf::Matrix3x3 m(q);
        m.getRPY(roll,pitch,theta_ref);

        args["p"]((n_states + n_controls) * n + 3) = rbt_ref_traj_msg.poses[n].pose.position.x;
        args["p"]((n_states + n_controls) * n + 4) = rbt_ref_traj_msg.poses[n].pose.position.y;
        args["p"]((n_states + n_controls) * n + 5) = theta_ref;
        args["p"]((n_states + n_controls) * n + 6) = u_ref;
        args["p"]((n_states + n_controls) * n + 7) = omega_ref;
    }
}
void mpc_ros1_rbt1::make_args_p()
{
    casadi::DM p = casadi::DM::zeros((n_states + n_controls) * N + n_states,1);

    args.insert(std::pair<std::string,casadi::DM>("p",p));

    for(int idx = 0; idx < n_states; ++idx)
    {
        args["p"](idx) = rbt_state_init(idx);
    }
}

void mpc_ros1_rbt1::rbt_odom_Callbck(const nav_msgs::Odometry::ConstPtr& msg)
{
    buf.lock();
    rbt_odom_queue.push(*msg);
    buf.unlock();
}

void mpc_ros1_rbt1::rbt_ref_traj_Callbck(const nav_msgs::Path::ConstPtr& msg)
{
    buf.lock();
    rbt_ref_traj_queue.push(*msg);
    buf.unlock();
}

void mpc_ros1_rbt1::avoid_obstacle()
{
    double rob_diam = 0.40;
    double safe_dis = 0.1;

    lbg = casadi::DM::zeros(n_states * (N + 1) + n_obstacles * (N + 1), 1);
    ubg = casadi::DM::inf(n_states * (N + 1) + n_obstacles * (N + 1), 1);

    for (int idx = 0; idx < n_states * (N + 1); ++idx)
    {
        lbg(idx) = 0;
        ubg(idx) = 0;
    }

    for (int k = 0; k < N + 1; ++k)
    {
        for (int j = 0; j < n_obstacles; ++j)
        {
            // 최소 거리 = 로봇의 크기 + 장애물의 지름/2 + 안전거리
            double min_distance = (rob_diam + obstacle_vec[j][2]) / 2 + safe_dis;

            casadi::SX dist = casadi::SX::sqrt(casadi::SX::pow(X(0, k) - obstacle_vec[j][0], 2) +
                                               casadi::SX::pow(X(1, k) - obstacle_vec[j][1], 2));
            int idx = n_states * (N + 1) + j + n_obstacles * k;
            g = casadi::SX::vertcat({g, dist});
            lbg(idx) = min_distance;
            ubg(idx) = casadi::inf;
        }
    }

    args["lbg"] = lbg;
    args["ubg"] = ubg;
}

void mpc_ros1_rbt1::reset()
{
    rbt_pre_msg.poses.clear();
    rbt_ref_msg.poses.clear();

    ++loop_count;

    rbt_odom_in = false;
    rbt_ref_traj_in = false;
}

void mpc_ros1_rbt1::publish()
{
    rbt_cmd_pub.publish(rbt_cmd_msg);
    rbt_pre_pub.publish(rbt_pre_msg);
}

void mpc_ros1_rbt1::cmd_vel_msg_set()
{
    rbt_cmd_msg.linear.x = (double)U0(0,0);
    rbt_cmd_msg.angular.z = (double)U0(1,0);
}

void mpc_ros1_rbt1::predictive_traj_msg_set()
{
    rbt_pre_msg.header.frame_id = map_id;
    rbt_pre_msg.header.stamp = ros::Time::now();
    for(int n = 0; n < N; ++n)
    {
        geometry_msgs::PoseStamped n_path;
        n_path.pose.position.x = (double)X0(0,n);
        n_path.pose.position.y = (double)X0(1,n);

        tf::Quaternion n_q;
        n_q.setRPY(0.0, 0.0, pi_to_pi((double)X0(2,n)));

        n_path.pose.orientation.x = n_q.x();
        n_path.pose.orientation.y = n_q.y();
        n_path.pose.orientation.z = n_q.z();
        n_path.pose.orientation.w = n_q.w();

        rbt_pre_msg.poses.push_back(n_path);
    }
}

void mpc_ros1_rbt1::shift()
{
    casadi::DM X0_last = casadi::DM::vertcat({X0(0,-1),X0(1,-1),X0(2,-1)});
    X0 = casadi::DM::reshape(X0,X0.size1()*X0.size2(),1);
    X0 = casadi::DM::vertcat({X0(casadi::Slice(n_states,X0.size1()*X0.size2())),X0_last});
    X0 = casadi::DM::reshape(X0,n_states,N+1);

    casadi::DM U0_last = casadi::DM::vertcat({U0(0,-1),U0(1,-1)});
    U0 = casadi::DM::reshape(U0,U0.size1()*U0.size2(),1);
    U0 = casadi::DM::vertcat({U0(casadi::Slice(n_controls,U0.size1()*U0.size2())),U0_last});

    U0 = casadi::DM::reshape(U0,n_controls,N);
}

void mpc_ros1_rbt1::get_result()
{
    casadi::Slice slice_x0(0,n_states*(N+1));
    casadi::DM X0_temp = solver_result["x"](slice_x0);
    X0 = casadi::DM::reshape(X0_temp,n_states,N+1);

    casadi::Slice slice_u0(n_states*(N+1),solver_result["x"].size1());
    casadi::DM U0_temp = solver_result["x"](slice_u0);
    U0 = casadi::DM::reshape(U0_temp,n_controls,N);
}

void mpc_ros1_rbt1::call_solver()
{
    solver_result = solver(args);
}

void mpc_ros1_rbt1::reshape_and_init_opt_variable()
{
    args["x0"] = casadi::DM::vertcat({casadi::DM::reshape(X0,n_states * (N+1),1) , 
                                                  casadi::DM::reshape(U0,n_controls * N,1)});
}

void mpc_ros1_rbt1::setting_solver()
{
    casadi::SX obj = 0;
    casadi::SXVector g_vec;
    for(int i = 0; i < n_states; ++i)
    {
        g_vec.emplace_back(X(i,0) - P(i));
        //std::cout << " value : " << X(i,0) - P(i) << std::endl;
    }
    g = casadi::SX::vertcat(g_vec);

    for(int n = 0; n < N; ++n)
    {
        casadi::SXVector st_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n));
        }
        casadi::SX st = casadi::SX::vertcat(st_vec);

        casadi::SXVector con_vec;
        for(int idx = 0; idx < n_controls; ++idx)
        {
            con_vec.emplace_back(U(idx,n));
        }
        casadi::SX con = casadi::SX::vertcat(con_vec);

        casadi::SXVector p_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            p_vec.emplace_back(st(idx) - P((n_states + n_controls) * n + 3 + idx,0));
        }
        casadi::SX state_err = casadi::SX::vertcat(p_vec);

        p_vec.clear();
        for(int idx = 0; idx < n_controls; ++idx)
        {
            p_vec.emplace_back(con(idx) - P((n_states + n_controls) * n + 6 + idx,0));
        }
        casadi::SX con_err = casadi::SX::vertcat(p_vec);

        if(n >= N - 1)
        {
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err) + casadi::SX::mtimes(casadi::SX::mtimes(con_err.T(),R),con_err);
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q_ter),state_err);
        }
        else
        {
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err) + casadi::SX::mtimes(casadi::SX::mtimes(con_err.T(),R),con_err);
        }

        st_vec.clear();
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n+1));
        }
        casadi::SX st_next = casadi::SX::vertcat(st_vec);
        casadi::SXVector f_value_vec = f(casadi::SXVector{st,con});
        casadi::SX f_value = casadi::SX::vertcat(f_value_vec);
        casadi::SX st_next_euler = st + (dt * f_value);
        g = casadi::SX::vertcat({g,st_next - st_next_euler});
    }

    if(n_obstacles != 0){
        //std::cout << "obstacles avoid" << std::endl;
        avoid_obstacle();
    }

    casadi::SX OPT_variables = casadi::SX::vertcat({casadi::SX::reshape(X,-1,1),casadi::SX::reshape(U,-1,1)});
    casadi::SXDict nlp_prob;

    nlp_prob.insert(std::pair<std::string,casadi::SX>("f",obj));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("x",OPT_variables));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("g",g));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("p",P));

    casadi::Dict opts = {
    {"ipopt", casadi::Dict{
        {"max_iter", 200},
        {"print_level", 0},
        {"acceptable_tol", 1e-8},
        {"acceptable_obj_change_tol", 1e-6}
    }},
    {"print_time", 0}
    };
    solver = casadi::nlpsol("solver","ipopt",nlp_prob,opts);
}

void mpc_ros1_rbt1::state_update()
{
    double rbt_yaw = pi_to_pi(q_to_e(rbt_odom_msg));
    rbt_state_init = casadi::DM::vertcat({rbt_odom_msg.pose.pose.position.x, rbt_odom_msg.pose.pose.position.y, rbt_yaw});
    std::cout << " X : " << rbt_odom_msg.pose.pose.position.x << std::endl;
    std::cout << " Y : " << rbt_odom_msg.pose.pose.position.y << std::endl;
    std::cout << " Theta : " << rbt_yaw << std::endl;
    std::cout << " ========= " << std::endl;
}

void mpc_ros1_rbt1::rbt_ref_traj_update()
{
    if (!rbt_ref_traj_queue.empty())
    {
        rbt_ref_traj_msg = rbt_ref_traj_queue.front();
        rbt_ref_traj_queue.pop();
        rbt_ref_traj_in = true;
    }
}

void mpc_ros1_rbt1::rbt_odom_update()
{
    if (!rbt_odom_queue.empty())
    {
        rbt_odom_msg = rbt_odom_queue.front();
        rbt_odom_queue.pop();
        rbt_odom_in = true;
    }
}

void mpc_ros1_rbt1::obstacle_set()
{
    std::vector<double> obstacle1 = {1.0, -0.1, 0.1};
    std::vector<double> obstacle2 = {4.0, 0.1, 0.1};
    std::vector<double> obstacle3 = {7.0, -0.1, 0.1};
    std::vector<double> obstacle4 = {10.0, 0.1, 0.1};
    //obstacle_vec.emplace_back(obstacle1);
    //obstacle_vec.emplace_back(obstacle2);
    //obstacle_vec.emplace_back(obstacle3);
    //obstacle_vec.emplace_back(obstacle4);
    n_obstacles = static_cast<int>(obstacle_vec.size());
}

void mpc_ros1_rbt1::mpc_init()
{
    x      = casadi::SX::sym("x");
    y      = casadi::SX::sym("y");
    theta  = casadi::SX::sym("theta");
    states = casadi::SX::vertcat({x, y, theta});
    n_states = states.numel();

    v        = casadi::SX::sym("v");
    omega    = casadi::SX::sym("omega");
    controls = casadi::SX::vertcat({v, omega});
    n_controls = controls.numel();

    rhs = casadi::SX::vertcat({v * casadi::SX::cos(theta),
                           v * casadi::SX::sin(theta),
                           omega});

    f = casadi::Function("f", {states, controls}, {rhs});
    U = casadi::SX::sym("U", n_controls, N);
    X = casadi::SX::sym("X", n_states, N + 1);
    P = casadi::SX::sym("P", n_states + N * (n_states + n_controls));

    Q = casadi::SX::diagcat({Q_x, Q_y, Q_theta});
    R = casadi::SX::diagcat({R1, R2});

    // Terminal
    Q_ter = casadi::SX::diagcat({Q_x * Q_terminor, Q_y * Q_terminor, Q_theta * Q_terminor});

    lbg = casadi::DM::zeros(n_states * (N + 1), 1);
    ubg = casadi::DM::zeros(n_states * (N + 1), 1);

    lbx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);
    ubx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);

    for (int idx = 0; idx < n_states * (N + 1); ++idx)
    {
        if (idx % n_states == 0)
        {
            lbx(idx) = x_min;
            ubx(idx) = x_max;
        }
        else if (idx % n_states == 1)
        {
            lbx(idx) = y_min;
            ubx(idx) = y_max;
        }
        else if (idx % n_states == 2)
        {
            lbx(idx) = theta_min;
            ubx(idx) = theta_max;
        }
    }

    bool idx_even = (N + 1) % 2 == 0;

    for (int idx = n_states * (N + 1); idx < lbx.size1(); ++idx)
    {
        if (idx_even)
        {
            if (idx % n_controls == 0)
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
            else
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
        }
        else
        {
            if (idx % n_controls == 0)
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
            else
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
        }
    }

    args.insert(std::make_pair("lbg", lbg));
    args.insert(std::make_pair("ubg", ubg));
    args.insert(std::make_pair("lbx", lbx));
    args.insert(std::make_pair("ubx", ubx));

    rbt_state_init = casadi::DM::vertcat({0.0, 0.0, 0.0});
    U0 = casadi::DM::zeros(N, n_controls);
    X0 = casadi::DM::repmat(rbt_state_init, 1, N + 1);
}

void mpc_ros1_rbt1::node_param_init()
{
    //int N;
    //double dt, v_max, v_min;
    std::string ref_topic_name, odom_topic_name, cmd_topic_name, pre_topic_name, map_id_name;

    
    nh_.getParam("/mpc_ros1_rbt1_node/N", N);
    nh_.getParam("/mpc_ros1_rbt1_node/dt", dt);
    nh_.getParam("/mpc_ros1_rbt1_node/v_max", v_max);
    nh_.getParam("/mpc_ros1_rbt1_node/v_min", v_min);
    nh_.getParam("/mpc_ros1_rbt1_node/cmd_topic_name", cmd_topic_name);
    nh_.getParam("/mpc_ros1_rbt1_node/pose_topic_name", odom_topic_name);
    nh_.getParam("/mpc_ros1_rbt1_node/pre_topic_name", pre_topic_name);
    nh_.getParam("/mpc_ros1_rbt1_node/ref_topic_name", ref_topic_name);
    nh_.getParam("/mpc_ros1_rbt1_node/frame_id", map_id_name);
    
    N = N;
    dt = dt;
    v_max = v_max;
    v_min = v_min;
    ref_topic = ref_topic_name;
    odom_topic = odom_topic_name;
    cmd_topic = cmd_topic_name;
    pre_topic = pre_topic_name;
    map_id = map_id_name;

    std::cout << "// ================================================ //" << std::endl;
    std::cout << "// Node            : " << N << std::endl;
    std::cout << "// dt              : " << dt << std::endl;
    std::cout << "// v_max           : " << v_max << std::endl;
    std::cout << "// v_min           : " << v_min << std::endl;
    std::cout << "// ref_topic_name  : " << ref_topic << std::endl;
    std::cout << "// pose_topic_name : " << odom_topic << std::endl;
    std::cout << "// cmd_topic_name  : " << cmd_topic << std::endl;
    std::cout << "// pre_topic_name  : " << pre_topic << std::endl;
    std::cout << "// ================================================ //" << std::endl;
}

void mpc_ros1_rbt1::init()
{
    rbt_odom_sub = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 1, &mpc_ros1_rbt1::rbt_odom_Callbck, this);
    rbt_ref_traj_sub = nh_.subscribe<nav_msgs::Path>(ref_topic, 1, &mpc_ros1_rbt1::rbt_ref_traj_Callbck, this);

    rbt_cmd_pub = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    rbt_pre_pub = nh_.advertise<nav_msgs::Path>(pre_topic, 1);
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
}

void mpc_ros1_rbt1::run()
{

    obstacle_set();
    mpc_init();

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rbt_odom_update();
        rbt_ref_traj_update();
        if (rbt_odom_in && rbt_ref_traj_in)
        {
            state_update();
            setting_solver();
            make_args_p();
            setting_reference();
            reshape_and_init_opt_variable();
            call_solver();
            get_result();
            cmd_vel_msg_set();
            predictive_traj_msg_set();
            publish();
            shift();
            reset();
            casadi::Dict solver_iter = solver.stats();
            std::cout << "==================================" << std::endl;
            //std::cout << "loop time         : " << time << std::endl;
            std::cout << "loop num          : " << loop_count << std::endl;
            std::cout << "solver iteration  : " << solver_iter["iter_count"] << std::endl;
            std::cout << "solver cost       : " << solver_result["f"] << std::endl;
            std::cout << "robot heading(Rad): " << rbt_state_init(2) << std::endl;
            std::cout << "robot velocity    : " << rbt_cmd_msg.linear.x << std::endl;

        }

        if (!sub_print_once && (!rbt_odom_in || !rbt_ref_traj_in))
        {
            std::cout << "robot odom && trajectory not subscribed..." << std::endl;
            sub_print_once = true;
        }

        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_ros1_rbt1");
    mpc_ros1_rbt1 mpc_node;
    mpc_node.node_param_init();
    mpc_node.init();
    mpc_node.run();

    return 0;
}