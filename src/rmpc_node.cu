#include <vector>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <utility>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cuda_runtime.h>
#include "problem.cuh"
#include "rmpc/controller.cuh"

using namespace std;


class RMPC: public rclcpp::Node{

    struct Waypoint {
    float x;
    float y;
    float s;       // Accumulated distance along the track from the start
    float heading; // Tangent angle of the track at this point
    float speed;     // Add this to catch the CSV speed column
    float curvature; // Add this so the Frenet math knows how the track bends
    float l_bound;
    float r_bound;
};

struct FrenetState {
    float s;
    float d;
    float yaw_error;
};


std::vector<Waypoint> reference_path_; // This is laoded from CSV
    string odom_topic ;
    string drive_topic;
    string scan_topic;
    string grid_topic;
    // RHC params
    int horizon;
    double dt;
    //  Dynamic Lookahead & Velocity Tuning Parameters 
    double lookahead_distance; // Will now be overwritten dynamically
    double k_ld;               // Lookahead gain (seconds of forward travel)
    double lookahead_min ;      // Minimum lookahead for tight corners
    double lookahead_max;      // Maximum lookahead for high-speed straights
    
    double v_max ;              // Maximum straightaway speed
    double v_min ;              // Minimum cornering speed
    double wheel_base ; 
    
    double max_steering_limit;
    double max_steering_vel;
    int last_closest_idx = 0;
    // Controller 
    std::unique_ptr<rmpc::Controller<rmpc_f1tenth::F1TenthGrid>> rmpc_ctrl_;
    float current_steering_ = 0.0f;
    float current_velocity_cmd_ = 0.0f;
    // Grid params
    double resolution = 0.05;
    double width = 3.0;
    double height = 3.0;
    int rows = static_cast<int>(height/resolution); // discretize to rows and ols
    int columns = static_cast<int>(width/resolution);
    int x_offset = 0; // offset because we wanted to sit at the first row and utilize complete fro longitudianl direction
    int y_offset = columns/2;
    std::vector<int8_t> occupancy_grid;
    double collision_penalty = 1000;
    bool map_initialized_ = false;
    // CUDA params
    int8_t* cuda_occupancy_grid_{nullptr};
    size_t allocation_size_ = rows * columns * sizeof(int8_t);
    rmpc_f1tenth::F1TenthGrid::Params problem_params;
    // Publishers and subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_ ;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;

public:
RMPC(const std::string &node_name) : rclcpp::Node(node_name){
    // Pub/sub declaraiton
        this->declare_parameter<string>("odom_topic", "/ego_racecar/odom" );
        this->declare_parameter<string>("drive_topic", "/drive" );
        this->declare_parameter<string>("scan_topic","/scan");
        this->declare_parameter<string>("grid_topic", "/local_map");
        this->declare_parameter<int>("horizon",40);
        this->declare_parameter<double>("dt",0.05);
        this->declare_parameter<double>("v_max", 6.5);
        this->declare_parameter<double>("v_min", 0.5);
        this->declare_parameter<double>("max_steering_limit", 0.45);
        this->declare_parameter<double>("max_steering_vel", 3.2);
        // Fetchign parameters from yaml
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("drive_topic",drive_topic);
        this->get_parameter("scan_topic", scan_topic);
        this->get_parameter("grid_topic", grid_topic);
        v_max = this->get_parameter("v_max").as_double();
        v_min = this->get_parameter("v_min").as_double();
        max_steering_limit = this->get_parameter("max_steering_limit").as_double();
        max_steering_vel = this->get_parameter("max_steering_vel").as_double();
        horizon = this->get_parameter("horizon").as_int();
        dt = this->get_parameter("dt").as_double();

   
    
    // ROS Publishers &  subscribers
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 1, [this](const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg){
        this->odomCallback(pose_msg);
      });
    // scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //   scan_topic, 1, [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg){
    //     this->scan_callback(scan_msg);
    //   });
      drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
      grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic, 10);


      std::string package_share = ament_index_cpp::get_package_share_directory("rmpc_f1tenth");
std::string default_path = package_share + "/waypoints/IMS_centerline.csv";

this->declare_parameter<std::string>("centerline_path", default_path);
std::string centerline_path = this->get_parameter("centerline_path").as_string();

loadCenterline(centerline_path);
    
    // Cinfiguratio napraemters
    rmpc::Config cfg;
    cfg.n_horizon = horizon;
    cfg.dt = dt;
    cfg.u_max = max_steering_limit;      // Max steering angle (radians)
    cfg.u_min = -max_steering_limit ;   // Min steering angle (radians)
    cfg.du_max = max_steering_vel;     // Max steering velocity (radians/sec)
    cfg.n_samples = 5000;      // Number of parallel GPU rollouts
    cfg.n_basis = 7;           // 7 DCT frequency basis curves
    cfg.warm_start = true;
    cfg.seed = 70997;
    // Problem parameters
    
     RCLCPP_INFO(this->get_logger(), "Horizon : %d\ndt : %.2f\n v_max : %f", cfg.n_horizon, cfg.dt, this->v_max);
    // --- F1Tenth Physical Constraints (from params.yaml) ---
problem_params.car.mass = 3.47f;       
problem_params.car.lf = 0.15875f;      
problem_params.car.lr = 0.17145f;      
problem_params.car.Iz = 0.04712f;      
problem_params.car.car_width = 0.2032f;
problem_params.car.vx = 3.0f;
// --- Environmental & Aero Constants ---
problem_params.car.g = 9.81f;
problem_params.car.rho = 1.225f; // Standard air density
problem_params.car.cl_f = 0.0f;  // No aero downforce on F1Tenth
problem_params.car.cl_r = 0.0f;

/// F1Tenth Pacejka Tire Parameters: // from examples/CarSteersing
// Format: {B, C, D, E}
// D = 0.523 matches simulator friction_coeff
// B = -6.0f and -7.0f match C_S_front and C_S_rear stiffness slopes
problem_params.car.tire_f = {-6.0f, 1.5f, 0.523f, -0.1f};
problem_params.car.tire_r = {-7.0f, 1.5f, 0.523f, -0.1f};

// Heading error: MUST be stiff enough to counter small yaw drifts
problem_params.car.w.lateral = {400.0f, 600.0f, 50.0f};   // lin_gain = 50 kills the deadband
problem_params.car.w.heading = {3500.0f, 250.0f, 500.0f}; // High heading stiffness

problem_params.car.w.terminal_lateral = {200.0f, 400.0f, 20.0f};
problem_params.car.w.terminal_heading = {1500.0f, 200.0f, 100.0f};

problem_params.car.w.progress = 0.5f;
problem_params.car.w.actuation = 0.05f;   // Small penalty on excessive steering
problem_params.car.w.rate = 5.0f;         // Damps steering jitter between cycles
problem_params.car.w.decay = 0.3f;        // 30% discount over horizon

// Tire force & wall boundaries scaled to F1Tenth
problem_params.car.w.force_f = {20.0f, 2.0f};
problem_params.car.w.force_r = {20.0f, 2.0f};
problem_params.car.w.grip_limit_scaler = 1.0f;

problem_params.car.w.bound = 1500.0f;       // Sharp penalty at track limits (+/- 1.1m)
problem_params.car.w.bound_margin = 0.08f; // 5 cm safety margin before wall
problem_params.car.w.max_bound_cost = 1000.0f;

problem_params.car.w.slip = 1000.0f;
problem_params.car.w.slip_threshold_f = 0.15f;
problem_params.car.w.slip_threshold_r = 0.15f;
problem_params.car.w.max_slip_cost = 5000.0f;
    auto result = rmpc::Controller<rmpc_f1tenth::F1TenthGrid>::create(cfg, problem_params);

if (result.isError()) {
    // result.error() returns a const char* explaining exactly what failed in the config
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CUDA MPC: %s", result.error());
} else {
    // Unpack the value and move the unique_ptr into your class member
    rmpc_ctrl_ = std::move(result.value());
    RCLCPP_INFO(this->get_logger(), "GPU RMPC Controller initialized successfully.");
}
      RCLCPP_INFO(this->get_logger(), "%s Node Initialsed...", node_name.c_str());

}

void loadCenterline(const std::string& file_path, float target_speed = 4.0f) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open centerline file: %s", file_path.c_str());
        return;
    }

    reference_path_.clear();

    struct RawPoint {
        float x;
        float y;
        float w_right;
        float w_left;
    };
    std::vector<RawPoint> raw_points;

    std::string line;
    // Parse all raw points, skipping comments and malformed lines
    while (std::getline(file, line)) {
        // Strip whitespace and skip empty lines or comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        std::string val;
        RawPoint pt;

        try {
            // Format: x_m, y_m, w_tr_right_m, w_tr_left_m
            std::getline(ss, val, ','); pt.x = std::stof(val);
            std::getline(ss, val, ','); pt.y = std::stof(val);
            std::getline(ss, val, ','); pt.w_right = std::stof(val);
            std::getline(ss, val, ','); pt.w_left = std::stof(val);
            raw_points.push_back(pt);
        } catch (...) {
            continue;
        }
    }

    const size_t N = raw_points.size();
    if (N < 3) {
        RCLCPP_ERROR(this->get_logger(), "Insufficient waypoints parsed: %zu", N);
        return;
    }

    reference_path_.resize(N);

    
   reference_path_[0].s = 0.0f;
    std::vector<float> segment_ds(N);

    // Compute segment lengths, headings, and accumulated 's' via euclideanDistance
    for (size_t i = 0; i < N; ++i) {
        size_t next_idx = (i + 1) % N;

        auto [dist, theta] = euclideanDistance(
            {static_cast<double>(raw_points[i].x), static_cast<double>(raw_points[i].y)},
            {static_cast<double>(raw_points[next_idx].x), static_cast<double>(raw_points[next_idx].y)}
        );

        segment_ds[i] = static_cast<float>(dist);
        reference_path_[i].heading = static_cast<float>(theta);

        // Accumulate distance along the track
        if (i + 1 < N) {
            reference_path_[i + 1].s = reference_path_[i].s + segment_ds[i];
        }
    }

   
    // Compute curvature using the cached segment lengths and assign waypoint fields
    for (size_t i = 0; i < N; ++i) {
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

    RCLCPP_INFO(this->get_logger(), 
                "Successfully loaded centerline! Points: %zu" 
                , N);
}
std::pair<double,double> euclideanDistance(const std::pair<double,double> &Point_A, const std::pair<double,double> &Point_B){
    double dx = Point_B.first - Point_A.first;
    double dy = Point_B.second - Point_A.second;
    double distance = std::hypot(dx, dy);
    double theta = std::atan2(dy, dx);

    return {distance,theta};
}
bool boundingBox(const int &x_index, const int &y_index){
    if(x_index < 0 || x_index >= this->columns || y_index < 0 || y_index >= this->rows)
            return false;
        return true;
}
std::pair<int,int> convertCartesianToGrid(const double &x_coordinate, const double &y_coordinate ){
    
    int x_index = static_cast<int>(x_coordinate / this->resolution) + this->x_offset;
    int y_index = static_cast<int>(y_coordinate / this->resolution) + this->y_offset;
    
    return {x_index,y_index};
}


// void visualize_occupancy_grid(const std_msgs::msg::Header& scan_header) {
//         nav_msgs::msg::OccupancyGrid grid_msg;

//         grid_msg.header.stamp = scan_header.stamp;
//         grid_msg.header.frame_id = scan_header.frame_id; 

//         grid_msg.info.resolution = this->resolution; 
//         grid_msg.info.width = this->columns;         
//         grid_msg.info.height = this->rows;           

//         grid_msg.info.origin.position.x = -(this->x_offset * this->resolution); 
//         grid_msg.info.origin.position.y = -(this->y_offset * this->resolution);
//         grid_msg.info.origin.position.z = 0.0;
//         grid_msg.info.origin.orientation.w = 1.0; 

        
//         grid_msg.data = this->occupancy_grid;
//         grid_publisher_->publish(grid_msg);

//         RCLCPP_DEBUG(this->get_logger(), "Published local occupancy grid. Resolution: %.2f", this->resolution);
//     }

// void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg){
//     this->occupancy_grid.assign(this-> rows * this-> columns, 0);

//     // Iterating through ranges
//     for(size_t i = 0 ; i < scan_msg->ranges.size() ; i++){
//         // filtering invalid data
//         if(isinf(scan_msg->ranges[i]) || isnan(scan_msg->ranges[i]) || scan_msg->ranges[i] > scan_msg->range_max)
//             continue;
//         // convertign the range idnex itn oan angle
//         float angle = scan_msg->angle_min + (i* scan_msg->angle_increment);  

//         // covnertign polar coordiantes to cartesioan cooridnates
//         float x_cart = scan_msg->ranges[i] * cos(angle);
//         float y_cart = scan_msg->ranges[i] * sin(angle);

//         // convertign cartesian coordianes to our matrix gird 
//         // int x_ind = static_cast<int>(x_cart / this->resolution) + this->x_offset;
//         // int y_ind = static_cast<int>(y_cart / this->resolution) + this->y_offset;
//         std::pair<int,int> grid_indices = convertCartesianToGrid(
//             (double) x_cart, (double) y_cart
//         );

//         // bodungin box 
//         if(boundingBox(grid_indices.first,grid_indices.second) == false)
//             continue;


//         // 2D Square Inflation
//         int inflation_radius = 5; 

//         for (int r = -inflation_radius; r <= inflation_radius; r++) {
//             for (int c = -inflation_radius; c <= inflation_radius; c++) {
                
//                 int inflate_x = grid_indices.first + r;
//                 int inflate_y = grid_indices.second + c;

//                 // Ensure the newly inflated cell is still inside the grid bounds
//                 if (inflate_x >= 0 && inflate_x < this->columns && inflate_y >= 0 && inflate_y < this->rows) {
                    
//                     // Flatten 2D to 1D only for the valid inflated cells
//                     int inflate_index = inflate_x  + (inflate_y * this->columns);
//                     this->occupancy_grid[inflate_index] = 100; // RVIZ requries 100
//                 }
//             }
//         }


//     }

//     // Visualise
//     visualize_occupancy_grid(scan_msg->header);

//     // Stream to GPU Memory

//     cudaMemcpy(cuda_occupancy_grid_, this->occupancy_grid.data(), 
//     this->occupancy_grid.size() * sizeof(int8_t), 
//     cudaMemcpyHostToDevice);
//    this-> map_initialized_ = true;
// }

void extractPose(const nav_msgs::msg::Odometry::ConstPtr &pose_msg, double &x_car, double &y_car, double &yaw, double &vx, double &vy, double &r)
    {   


        vx = pose_msg->twist.twist.linear.x;
        vy = pose_msg->twist.twist.linear.y;
        r  = pose_msg->twist.twist.angular.z;

        x_car = pose_msg->pose.pose.position.x;
        y_car = pose_msg->pose.pose.position.y;
        
        double q_x = pose_msg->pose.pose.orientation.x;
        double q_y = pose_msg->pose.pose.orientation.y; 
        double q_z = pose_msg->pose.pose.orientation.z;
        double q_w = pose_msg->pose.pose.orientation.w;

        tf2::Quaternion quart(q_x, q_y, q_z, q_w);
        tf2::Matrix3x3 mat(quart);

        double roll, pitch;
        mat.getRPY(roll, pitch, yaw);

        
    }
void globalToFrenet(float car_x, float car_y, float car_yaw, 
                      float &s, float &d, float &yaw_error, float &k) {
    if (reference_path_.empty()) return;

    
    int num_waypoints = reference_path_.size();

    int closest_idx = this->last_closest_idx;
    float min_dist = 1e9; 
    
    // Search strictly forward along the track (e.g., up to 30 waypoints ahead) 
    // plus a small backward buffer (-5) for minor slips.
    int search_back = 5;
    int search_ahead = 30;

    for (int i = -search_back; i <= search_ahead; ++i) {
        int idx = (this->last_closest_idx + i) % num_waypoints;
        if (idx < 0) idx += num_waypoints; 
        
        float dist = std::hypot(car_x - reference_path_[idx].x, car_y - reference_path_[idx].y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = idx;
        }
    }
    
    this->last_closest_idx = closest_idx;
    const Waypoint& wp = reference_path_[closest_idx];

    // Compute Cross-Track Error (d) and Progress (s)
    float dx = car_x - wp.x;
    float dy = car_y - wp.y;

    k = wp.curvature;

    d = dx * (-std::sin(wp.heading)) + dy * std::cos(wp.heading);
    s = wp.s + (dx * std::cos(wp.heading) + dy * std::sin(wp.heading));

    yaw_error = car_yaw - wp.heading;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

   
}
void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg){

    // Pose extraction, conversion from quaternion to euler angle
        double x_car, y_car, yaw, vx, vy, r;
        extractPose(pose_msg, x_car, y_car, yaw, vx, vy, r);

    // Convert the coordiantes to frenet coorrdiantes
    float s, d, yaw_error, k;
    globalToFrenet(x_car, y_car, yaw, s, d, yaw_error, k);

    // current state frenet coordiantes
    // Clamp speed to prevent Pacejka divide-by-zero NaNs
    float safe_vx = std::max(this->v_min, (float)vx);  
    problem_params.car.vx = safe_vx;

    rmpc_ctrl_->setParams(this->problem_params);
    rmpc_f1tenth::F1TenthGrid::State current_state = {s, d, yaw_error, vy, r};

    
    

    std::vector<rmpc_f1tenth::F1TenthGrid::Reference> reference_horizon(this->horizon);

    float track_length = reference_path_.back().s;
    size_t search_idx = this->last_closest_idx;
    size_t num_wp = reference_path_.size();

for (int step = 0; step < this->horizon; ++step) {
        float s_target = s + (step * safe_vx * dt);
        s_target = std::fmod(s_target, track_length);
        if (s_target < 0.0f) s_target += track_length;

        // Advance sequentially along waypoints from search_idx
        while (reference_path_[(search_idx + 1) % num_wp].s < s_target && 
               reference_path_[(search_idx + 1) % num_wp].s > reference_path_[search_idx].s) {
            search_idx = (search_idx + 1) % num_wp;
        }

        const auto& wp = reference_path_[search_idx];

        rmpc_f1tenth::F1TenthGrid::Reference ref;
        ref.d_target    = 0.0f;
        ref.left_bound  = wp.l_bound;
        ref.right_bound = -wp.r_bound;
        ref.curvature   = wp.curvature;

        reference_horizon[step] = ref;
    }

    // 3. Fire the GPU Solver
    // You will need to add `float current_steering_ = 0.0f;` to your private class variables
    // so you can feed the last commanded angle back into the solver as u0.


    auto solve_result = rmpc_ctrl_->solve(current_state, this->current_steering_, reference_horizon);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
    "State -> s: %.2f, d: %.2f, yaw_err: %.2f", s, d, yaw_error);
    // 4. Extract Command and Publish
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.header.frame_id = "base_link";

    if (solve_result.isError()) {
        // Fallback: If the GPU fails to find a valid trajectory, stop the car.
        RCLCPP_ERROR(this->get_logger(), "MPC Solve Failed: %s", solve_result.error());
        drive_msg.drive.speed = 0.0f;
        drive_msg.drive.steering_angle = 0.0f;
        current_velocity_cmd_ = 0.0f;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
    "Currently Zero angle and velocity");
    } else {
        float optimal_steer_solver = solve_result.value().nextInput();
        this->current_steering_ = optimal_steer_solver;
        
        // 1. Look ahead and find the sharpest curve in the upcoming prediction horizon
        float max_k = 0.0f;
        for (int k = 0; k < this->horizon ; ++k) {
            if (std::abs(reference_horizon[k].curvature) > max_k) {
                max_k = std::abs(reference_horizon[k].curvature);
            }
        }

        // 2. Physics check: Calculate maximum safe speed for that corner
        // Formula: v = sqrt(mu * g / k)
        float friction_coeff = 0.523f; // From your params.yaml
        float gravity = 9.81f;
        float safe_corner_speed = std::sqrt((friction_coeff * gravity) / (max_k+ 1e-4f));

        // 3. Clamp target velocity to your configured min/max speeds
        double effective_v_min = std::max(0.8, (double)this->v_min);
        double raw_target_velocity = std::clamp((double)safe_corner_speed, effective_v_min, (double)this->v_max);

        // 4. Slew-rate limiter (Smooth braking/acceleration)
        float max_accel_step = 2.0f * this-> dt; 
        if (raw_target_velocity > this->current_velocity_cmd_) {
            this->current_velocity_cmd_ = std::min((float)raw_target_velocity, this->current_velocity_cmd_ + max_accel_step);
        } else {
            // Allow harder braking (4x multiplier) so it can slow down in time for the corner
            float braking_multiplier = 6.0;
            this->current_velocity_cmd_ = std::max((float)raw_target_velocity, this->current_velocity_cmd_ - max_accel_step * braking_multiplier); 
        }

        // Publish
        drive_msg.drive.steering_angle = optimal_steer_solver;
        drive_msg.drive.speed = this->current_velocity_cmd_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
    "raw_target_vel: %.2f, curr_vel_cmd: %.2f, optimal_steer_solver: %.2f", raw_target_velocity, current_velocity_cmd_, optimal_steer_solver);
    }
    

    this->drive_publisher_->publish(drive_msg);
}
~RMPC(){
    RCLCPP_INFO(this->get_logger(), "Destroying Node...");
}
};

int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
    string node_name = "rmpc_node";
    auto node = std::make_shared<RMPC>(node_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}