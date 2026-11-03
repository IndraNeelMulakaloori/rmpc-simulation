#include "rmpc_f1tenth/rmpc_node.cuh"

namespace rmpc_f1tenth{
    RMPC::RMPC(const std::string &node_name) : rclcpp::Node(node_name)
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
        centerline_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(centerline_topic_, 10);
        traj_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(traj_topic_, 10);

        
        // Lidar occupancy grid parameters
        rows_ = static_cast<int>(height_ / resolution_); // discretize to rows and ols
        columns_ = static_cast<int>(width_ / resolution_);
        x_offset_ = 0; // offset because we wanted to sit at the first row and utilize complete longitudinal direction
        y_offset_ = rows_ / 2;
        allocation_size_ = rows_ * columns_ * sizeof(int8_t);
        occupancy_grid_.assign(rows_ * columns_, 0); // Initialize the CPU-side occupancy grid with zeros

        // Retrieve waypoints(Centerline)
        std::string package_share = ament_index_cpp::get_package_share_directory("rmpc_f1tenth");
        std::string default_path = package_share + "/waypoints/IMS_centerline.csv";
        this->declare_parameter<std::string>("centerline_path", default_path);
        std::string centerline_path = this->get_parameter("centerline_path").as_string();

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
        // Topic parameters
        this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("grid_topic", "/local_map");
        this->declare_parameter<std::string>("centerline_topic", "/centerline_marker");
        this->declare_parameter<std::string>("traj_topic", "/rmpc_trajectories");

        // Solver & Horizon parameters
        this->declare_parameter<int>("horizon", 100);
        this->declare_parameter<double>("dt", 0.02);
        this->declare_parameter<int>("samples", 5000);
        this->declare_parameter<int>("n_basis", 7);
        this->declare_parameter<int>("seed", 70997);

        // Vehicle & Limit parameters
        this->declare_parameter<double>("v_max", 10.5);
        this->declare_parameter<double>("v_min", 2.5);
        this->declare_parameter<double>("wheel_base", 0.3302);
        this->declare_parameter<double>("max_steering_limit", 0.4189);
        this->declare_parameter<double>("max_steering_vel", 3.2);
        this->declare_parameter<float>("friction_coeff", 0.523f);
        this->declare_parameter<float>("gravity", 9.81f);
        this->declare_parameter<float>("max_accel", 7.51f);
        this->declare_parameter<float>("max_decel", 8.26f);

        // Occupancy Grid parameters
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("width", 6.0);
        this->declare_parameter<double>("height", 3.0);
        this->declare_parameter<int>("inflation_radius", 3);
        this->declare_parameter<int>("lateral_clearance", 1);

        // Fetching parameters
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        // opp_odom_topic_ = this->get_parameter("opp_odom_topic").as_string();
        drive_topic_ = this->get_parameter("drive_topic").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        grid_topic_ = this->get_parameter("grid_topic").as_string();
        centerline_topic_ = this->get_parameter("centerline_topic").as_string();
        traj_topic_ = this->get_parameter("traj_topic").as_string();

        horizon_ = this->get_parameter("horizon").as_int();
        dt_ = this->get_parameter("dt").as_double();
        samples_ = this->get_parameter("samples").as_int();
        n_basis_ = this->get_parameter("n_basis").as_int();
        seed_ = this->get_parameter("seed").as_int();

        v_max_ = this->get_parameter("v_max").as_double();
        v_min_ = this->get_parameter("v_min").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        max_steering_limit_ = this->get_parameter("max_steering_limit").as_double();
        max_steering_vel_ = this->get_parameter("max_steering_vel").as_double();
        friction_coeff_ = static_cast<float>(this->get_parameter("friction_coeff").as_double());
        gravity_ = static_cast<float>(this->get_parameter("gravity").as_double());
        max_accel_ = static_cast<float>(this->get_parameter("max_accel").as_double());
        max_decel_ = static_cast<float>(this->get_parameter("max_decel").as_double());

        resolution_ = this->get_parameter("resolution").as_double();
        width_ = this->get_parameter("width").as_double();
        height_ = this->get_parameter("height").as_double();
        inflation_radius_ = this->get_parameter("inflation_radius").as_int();
        lateral_clearance_ = this->get_parameter("lateral_clearance").as_int();
    }

    void RMPC::initController()
    {
        /**
         * @brief Initialize the RMPC controller with the current configuration.
         */
        rmpc::Config cfg;
        cfg.n_horizon = horizon_;
        cfg.dt = dt_;
        cfg.u_max = max_steering_limit_;
        cfg.u_min = -max_steering_limit_;
        cfg.du_max = max_steering_vel_;
        cfg.n_samples = samples_;
        cfg.n_basis = n_basis_;
        cfg.warm_start = true;
        cfg.seed = seed_;

        // Allocate device memory for local occupancy grid
        cudaError_t err = cudaMalloc(&cuda_occupancy_grid_, allocation_size_);
        if (err != cudaSuccess)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to allocate cuda_occupancy_grid_: %s", cudaGetErrorString(err));
        }
        cudaMemset(cuda_occupancy_grid_, 0, allocation_size_);

        // Vehicle physical parameters
        problem_params_.car.mass = 3.47f;
        problem_params_.car.lf = 0.15875f;
        problem_params_.car.lr = 0.17145f;
        problem_params_.car.Iz = 0.04712f;
        problem_params_.car.car_width = 0.2032f;
        problem_params_.car.vx = static_cast<float>(v_min_);
        problem_params_.car.g = gravity_;
        problem_params_.car.rho = 1.225f;
        problem_params_.car.cl_f = 0.0f;
        problem_params_.car.cl_r = 0.0f;

        // Pacejka tire parameters
        problem_params_.car.tire_f = {-6.0f, 1.5f, friction_coeff_, -0.1f};
        problem_params_.car.tire_r = {-7.0f, 1.5f, friction_coeff_, -0.1f};

        // Tracking and regularization weights
        problem_params_.car.w.lateral = {400.0f, 600.0f, 50.0f};
        problem_params_.car.w.heading = {3500.0f, 250.0f, 500.0f};
        problem_params_.car.w.terminal_lateral = {200.0f, 400.0f, 20.0f};
        problem_params_.car.w.terminal_heading = {1500.0f, 200.0f, 100.0f};
        problem_params_.car.w.progress = 0.5f;
        problem_params_.car.w.actuation = 0.05f;
        problem_params_.car.w.rate = 5.0f;
        problem_params_.car.w.decay = 0.3f;

        // Constraints & Limits
        problem_params_.car.w.force_f = {20.0f, 2.0f};
        problem_params_.car.w.force_r = {20.0f, 2.0f};
        problem_params_.car.w.grip_limit_scaler = 1.0f;
        problem_params_.car.w.bound = 1500.0f;
        problem_params_.car.w.bound_margin = 0.08f;
        problem_params_.car.w.max_bound_cost = 1000.0f;
        problem_params_.car.w.slip = 1000.0f;
        problem_params_.car.w.slip_threshold_f = 0.15f;
        problem_params_.car.w.slip_threshold_r = 0.15f;
        problem_params_.car.w.max_slip_cost = 5000.0f;

        // Occupancy Grid problem binding
        problem_params_.hit_cost = 20000.0f;
        problem_params_.cuda_occupancy_grid = this->cuda_occupancy_grid_;
        problem_params_.resolution = static_cast<float>(this->resolution_);
        problem_params_.columns = this->columns_;
        problem_params_.rows = this->rows_;
        problem_params_.x_offset = this->x_offset_;
        problem_params_.y_offset = this->y_offset_;

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