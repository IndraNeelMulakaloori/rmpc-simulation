#include "rmpc_f1tenth/rmpc_node.cuh"

namespace rmpc_f1tenth{
    RMPC::RMPC(const std::string &node_name) : rclcpp::Node(node_name, 
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        /**
         * @brief Constructor for the RMPC node.
         * @param node_name{std::string} The name of the ROS2 node.
         */
        declareAndLoadParameters();

        // ROS Publishers &  subscribers
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 1, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
            { this->odomCallback(pose_msg); });
        //   opp_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //   opp_odom_topic_, 1, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg){
        //     this->oppOdomCallback(pose_msg);
        //   });
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 1, [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
            { this->scanCallback(scan_msg); });
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
        grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);


        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        centerline_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(centerline_topic_, qos_profile);
        traj_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(traj_topic_, qos_profile);


        // Lidar occupancy grid parameters
        rows_ = static_cast<int>(height_ / resolution_); // discretize to rows and cols
        columns_ = static_cast<int>(width_ / resolution_);
        x_offset_ = 0; // offset because we wanted to sit at the first row and utilize complete longitudinal direction
        y_offset_ = rows_ / 2;
        allocation_size_ = rows_ * columns_ * sizeof(int8_t);
        occupancy_grid_.assign(rows_ * columns_, 0); // Initialize the CPU-side occupancy grid with zeros

        // Load the centerline from the specified waypoints file
        // Current Script only supports centerline in the form of CSV with x,y,w_left,w_right
        // To support raceline.csv, 
        // we need to define a new method to handle the raceline format.
        std::string package_share = ament_index_cpp::get_package_share_directory("rmpc_f1tenth");
        std::string centerline_path = package_share + '/'+ waypoints_file_;
    
        loadCenterline(centerline_path);
        visualizeCenterline();

        // Initialize CUDA memory and RMPC Controller
        initController();

        RCLCPP_INFO(this->get_logger(), "%s Node Initialized successfully.", node_name.c_str());
    }

    RMPC::~RMPC()
    {
        if (cuda_occupancy_grid_ != nullptr)
        {
            cudaFree(cuda_occupancy_grid_);
            cuda_occupancy_grid_ = nullptr;
            RCLCPP_INFO(this->get_logger(), "Freed Occupancy grid from GPU");
        }

        RCLCPP_INFO(this->get_logger(), "Destroying Node...");
    }

    void RMPC::declareAndLoadParameters()
    {
        /**
         * @brief Declare and load all parameters for the RMPC node.
         */
        // Map parameters
        waypoints_file_ = this->get_parameter("waypoints_file").as_string();
        // Topic parameters
        odom_topic_       = this->get_parameter("odom_topic").as_string();
        // opp_odom_topic_ = this->get_parameter("opp_odom_topic").as_string();
        drive_topic_      = this->get_parameter("drive_topic").as_string();
        scan_topic_       = this->get_parameter("scan_topic").as_string();
        grid_topic_       = this->get_parameter("grid_topic").as_string();
        centerline_topic_ = this->get_parameter("centerline_topic").as_string();
        traj_topic_       = this->get_parameter("traj_topic").as_string();
        
        
        // Receding Horizon / Solver Parameters
        horizon_    = this->get_parameter("solver.horizon").as_int();
        dt_         = this->get_parameter("solver.dt").as_double();
        samples_    = this->get_parameter("solver.samples").as_int();
        n_basis_    = this->get_parameter("solver.n_basis").as_int();
        seed_       = this->get_parameter("solver.seed").as_int();
        warm_start_ = this->get_parameter("solver.warm_start").as_bool();
        
        //Occupancy Grid Parameters
        resolution_        = this->get_parameter("occupancy_grid.resolution").as_double();
        width_             = this->get_parameter("occupancy_grid.width").as_double();
        height_            = this->get_parameter("occupancy_grid.height").as_double();
        inflation_radius_  = this->get_parameter("occupancy_grid.inflation_radius").as_int();
        lateral_clearance_ = this->get_parameter("occupancy_grid.lateral_clearance").as_int();
        hit_cost_          = static_cast<float>(this->get_parameter("occupancy_grid.hit_cost").as_double());

        // Speed & Actuation Limits
        v_max_              = this->get_parameter("speed_limits.v_max").as_double();
        v_min_              = this->get_parameter("speed_limits.v_min").as_double();
        wheel_base_         = this->get_parameter("speed_limits.wheel_base").as_double();
        max_steering_limit_ = this->get_parameter("speed_limits.max_steering_limit").as_double();
        max_steering_vel_   = this->get_parameter("speed_limits.max_steering_vel").as_double();
        friction_coeff_     = static_cast<float>(this->get_parameter("speed_limits.friction_coeff").as_double());
        gravity_            = static_cast<float>(this->get_parameter("speed_limits.gravity").as_double());
        max_accel_          = static_cast<float>(this->get_parameter("speed_limits.max_accel").as_double());
        max_decel_          = static_cast<float>(this->get_parameter("speed_limits.max_decel").as_double());

        
    }

    void RMPC::initController()
    {
        /**
         * @brief Initialize the RMPC controller with the current configuration.
         */

        
        rmpc::Config cfg;
        cfg.n_horizon  = horizon_;
        cfg.dt         = dt_;
        cfg.u_max      = max_steering_limit_;
        cfg.u_min      = -(max_steering_limit_);
        cfg.du_max     = max_steering_vel_;
        cfg.n_samples  = samples_;
        cfg.n_basis    = n_basis_;
        cfg.warm_start = warm_start_;
        cfg.seed       = seed_;

        // Allocate device memory for local occupancy grid
        cudaError_t err = cudaMalloc(&cuda_occupancy_grid_, allocation_size_);
        if (err != cudaSuccess)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate cuda_occupancy_grid_: %s", cudaGetErrorString(err));
        }
        cudaMemset(cuda_occupancy_grid_, 0, allocation_size_);
    

        
        // Vehicle physical parameters
        // Load vehicle parameters from the ROS2 parameter server
        std::map<std::string, rclcpp::Parameter> vehicle_params;
        this->get_node_parameters_interface()->get_parameters_by_prefix("vehicle_dimensions", vehicle_params);

        problem_params_.car.mass      = static_cast<float>(vehicle_params["mass"].as_double());
        problem_params_.car.lf        = static_cast<float>(vehicle_params["lf"].as_double());
        problem_params_.car.lr        = static_cast<float>(vehicle_params["lr"].as_double());
        problem_params_.car.Iz        = static_cast<float>(vehicle_params["Iz"].as_double());
        problem_params_.car.car_width = static_cast<float>(vehicle_params["car_width"].as_double());
        problem_params_.car.vx        = static_cast<float>(v_min_);
        problem_params_.car.g         = gravity_;
        problem_params_.car.rho       = static_cast<float>(vehicle_params["rho"].as_double());
        problem_params_.car.cl_f      = static_cast<float>(vehicle_params["cl_f"].as_double());
        problem_params_.car.cl_r      = static_cast<float>(vehicle_params["cl_r"].as_double());

        // Pacejka tire parameters
        auto tf = vehicle_params["tire_f_bce"].as_double_array();
        auto tr = vehicle_params["tire_r_bce"].as_double_array();
        problem_params_.car.tire_f = {static_cast<float>(tf[0]), static_cast<float>(tf[1]), friction_coeff_, static_cast<float>(tf[2])};
        problem_params_.car.tire_r = {static_cast<float>(tr[0]), static_cast<float>(tr[1]), friction_coeff_, static_cast<float>(tr[2])};
        

        // Tracking and regularization weights
        std::map<std::string, rclcpp::Parameter> weights_params_;
        this->get_node_parameters_interface()->get_parameters_by_prefix("weights", weights_params_);
        auto w_lat  = weights_params_["lateral"].as_double_array();
        auto w_head = weights_params_["heading"].as_double_array();
        auto wt_lat = weights_params_["terminal_lateral"].as_double_array();
        auto wt_hd  = weights_params_["terminal_heading"].as_double_array();
        problem_params_.car.w.lateral = {static_cast<float>(w_lat[0]), static_cast<float>(w_lat[1]), static_cast<float>(w_lat[2])};
        problem_params_.car.w.heading = {static_cast<float>(w_head[0]), static_cast<float>(w_head[1]), static_cast<float>(w_head[2])};
        problem_params_.car.w.terminal_lateral = {static_cast<float>(wt_lat[0]), static_cast<float>(wt_lat[1]), static_cast<float>(wt_lat[2])};
        problem_params_.car.w.terminal_heading = {static_cast<float>(wt_hd[0]), static_cast<float>(wt_hd[1]), static_cast<float>(wt_hd[2])};
        problem_params_.car.w.progress = static_cast<float>(weights_params_["progress"].as_double());
        problem_params_.car.w.actuation = static_cast<float>(weights_params_["actuation"].as_double());
        problem_params_.car.w.rate = static_cast<float>(weights_params_["rate"].as_double());
        problem_params_.car.w.decay = static_cast<float>(weights_params_["decay"].as_double());

        // Constraints & Limits
        auto f_f = weights_params_["force_f"].as_double_array();
        auto f_r = weights_params_["force_r"].as_double_array();
        problem_params_.car.w.force_f           = {static_cast<float>(f_f[0]), static_cast<float>(f_f[1])};
        problem_params_.car.w.force_r           = {static_cast<float>(f_r[0]), static_cast<float>(f_r[1])};
        problem_params_.car.w.grip_limit_scaler = static_cast<float>(weights_params_["grip_limit_scaler"].as_double());
        problem_params_.car.w.bound             = static_cast<float>(weights_params_["bound"].as_double());
        problem_params_.car.w.bound_margin      = static_cast<float>(weights_params_["bound_margin"].as_double());
        problem_params_.car.w.max_bound_cost    = static_cast<float>(weights_params_["max_bound_cost"].as_double());
        problem_params_.car.w.slip              = static_cast<float>(weights_params_["slip"].as_double());
        problem_params_.car.w.slip_threshold_f  = static_cast<float>(weights_params_["slip_threshold_f"].as_double());
        problem_params_.car.w.slip_threshold_r  = static_cast<float>(weights_params_["slip_threshold_r"].as_double());
        problem_params_.car.w.max_slip_cost     = static_cast<float>(weights_params_["max_slip_cost"].as_double());

        // Occupancy Grid problem binding
        problem_params_.resolution = static_cast<float>(resolution_);
        problem_params_.columns = columns_;
        problem_params_.rows = rows_;
        problem_params_.x_offset = x_offset_;
        problem_params_.y_offset = y_offset_;
        problem_params_.cuda_occupancy_grid = this->cuda_occupancy_grid_;
        problem_params_.hit_cost = hit_cost_;

        auto result = rmpc::Controller<F1TenthGrid>::create(cfg, problem_params_);
        if (result.isError())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CUDA MPC: %s", result.error());
        }
        else
        {
            this->rmpc_ctrl_ = std::move(result.value());
            RCLCPP_INFO(this->get_logger(), "GPU RMPC Controller initialized successfully.");
        }
    }

    SegmentVector RMPC::euclideanDistance(double x1, double y1, double x2, double y2) const
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return SegmentVector{std::hypot(dx, dy), std::atan2(dy, dx)};
    }

    void RMPC::loadCenterline(const std::string &file_path, float target_speed)
    {
        /**
         * @brief Load the centerline file and populate the reference path.
         * @param file_path{string} The path to the centerline CSV file.
         * @param target_speed{float} The target speed for the reference path.
         */

        // Open the centerline file
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open centerline file: %s", file_path.c_str());
            return;
        }
        // Reset the reference path before loading new data
        reference_path_.clear();

        // Read and parse the centerline data
        std::vector<RawPoint> raw_points;
        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            std::stringstream ss(line);
            std::string val;
            RawPoint pt;

            try
            {
                std::getline(ss, val, ',');
                pt.x = std::stof(val);
                std::getline(ss, val, ',');
                pt.y = std::stof(val);
                std::getline(ss, val, ',');
                pt.w_right = std::stof(val);
                std::getline(ss, val, ',');
                pt.w_left = std::stof(val);
                raw_points.push_back(pt);
            }
            catch (...)
            {
                continue;
            }
        }
        // If the number of points is less than 3, log an error and return
        const size_t N = raw_points.size();
        if (N < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Insufficient waypoints parsed: %zu", N);
            return;
        }

        // Resize the reference path and initialize the first waypoint's s value
        reference_path_.resize(N);
        reference_path_[0].s = 0.0f;
        std::vector<float> segment_ds(N);

        // Iterate through the raw points to compute distances, headings, and curvatures
        for (size_t i = 0; i < N; ++i)
        {
            size_t next_idx = (i + 1) % N;

            SegmentVector seg = euclideanDistance(
                raw_points[i].x, raw_points[i].y,
                raw_points[next_idx].x, raw_points[next_idx].y);

            segment_ds[i] = static_cast<float>(seg.distance);
            reference_path_[i].heading = static_cast<float>(seg.angle);

            if (i + 1 < N)
            {
                reference_path_[i + 1].s = reference_path_[i].s + segment_ds[i];
            }
        }

        // Compute the curvature for each segment
        for (size_t i = 0; i < N; ++i)
        {
            size_t next_idx = (i + 1) % N;

            float d_yaw = reference_path_[next_idx].heading - reference_path_[i].heading;
            d_yaw = std::atan2(std::sin(d_yaw), std::cos(d_yaw));

            float ds = segment_ds[i];
            reference_path_[i].curvature = (ds > 1e-4f) ? (d_yaw / ds) : 0.0f;
            reference_path_[i].x = raw_points[i].x;
            reference_path_[i].y = raw_points[i].y;
            reference_path_[i].speed = target_speed;
            reference_path_[i].l_bound = raw_points[i].w_left;
            reference_path_[i].r_bound = raw_points[i].w_right;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded track centerline with %zu points. Total Length: %.2f m",
                    N, reference_path_.back().s);
    }

} // namespace rmpc_f1tenth

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    std::string node_name = "rmpc_node";
    auto node = std::make_shared<rmpc_f1tenth::RMPC>(node_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}