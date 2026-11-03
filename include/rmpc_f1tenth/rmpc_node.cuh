#pragma once

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cuda_runtime.h>
#include "problem.cuh"
#include "rmpc/controller.cuh"

namespace rmpc_f1tenth
{

    /**
     * @brief
     * rmpc_node.cuh includes all Data structures and member functions for modular
     * and efficent simulation of RMPC controller
     */
    struct Waypoint
    {
        float x;
        float y;
        float s;         // Accumulated arc-length distance along track [m]
        float heading;   // Track tangent angle [rad]
        float speed;     // Reference speed [m/s]
        float curvature; // Track curvature kappa [1/m]
        float l_bound;   // Left track boundary distance [m]
        float r_bound;   // Right track boundary distance [m]
    };
    struct RawPoint
    {
        float x;
        float y;
        float w_right;
        float w_left;
    };
    struct CartesianState
    {
        double x;
        double y;
        double yaw;
        double vx;
        double vy;
        double yaw_rate;
    };

    struct FrenetState
    {
        float s;
        float d;
        float yaw_error;
        float curvature;
        int closest_idx;
    };

    struct GridCell
    {
        int x_idx;
        int y_idx;
    };

    struct SegmentVector
    {
        double distance;
        double angle;
    };

    struct DriveCommand
    {
        float steering_angle;
        float speed;
    };

    /**
     * ROS2 RMPC Node class
     */
    class RMPC : public rclcpp::Node
    {
        // Constructor & Destructor(overrides at runtime)
    public:
        explicit RMPC(const std::string &node_name);
        ~RMPC() override;

    private:
        /**
         * Data members of RMPC Node class
         */
        // Reference Track
        std::vector<Waypoint> reference_path_;
        int ego_closest_idx_{0};
        int opp_closest_idx_{0};

        // Topic Names
        std::string odom_topic_;
        std::string drive_topic_;
        std::string scan_topic_;
        std::string grid_topic_;
        std::string centerline_topic_;
        std::string traj_topic_;

        // RHC / Solver Parameters
        int horizon_{100};
        double dt_{0.02};
        int samples_{5000};
        int n_basis_{7};
        int seed_{70997};
        bool warm_start_{true};

        // Vehicle & Longitudinal Profile Parameters
        double v_max_{10.5};
        double v_min_{2.5};
        double wheel_base_{0.3302};
        double max_steering_limit_{0.4189};
        double max_steering_vel_{3.2};
        float friction_coeff_{0.523f};
        float gravity_{9.81f};
        float max_accel_{7.51f};
        float max_decel_{8.26f};

        // Active Control State
        float current_steering_{0.0f};
        float current_velocity_cmd_{0.0f};
        float min_forward_dist_{100.0f};

        // Local Occupancy Grid Parameters
        double resolution_{0.05};
        double width_{6.0};
        double height_{3.0};
        int rows_{0};
        int columns_{0};
        int x_offset_{0};
        int y_offset_{0};
        int inflation_radius_{3};
        int lateral_clearance_{1};
        float hit_cost_{0.0f};
        std::vector<int8_t> occupancy_grid_;
        bool map_initialized_{false};

        // CUDA & RMPC Solver Handles
        int8_t *cuda_occupancy_grid_{nullptr};
        size_t allocation_size_{0};
        F1TenthGrid::Params problem_params_{};
        std::unique_ptr<rmpc::Controller<F1TenthGrid>> rmpc_ctrl_;

        // ROS 2 Subscriptions & Publishers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_subscriber_; // Static ground truth obstacle
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centerline_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_publisher_;

        /**
         * Member functions of RMPC Node class
         */

        // Initialization (rmpc_node.cu)
        void declareAndLoadParameters();
        void initController();
        void loadCenterline(const std::string &file_path, float target_speed = 4.0f);
        SegmentVector euclideanDistance(double x1, double y1, double x2, double y2) const;

        // Perception & Occupancy Grid (perception.cu)
        void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg);
        GridCell convertCartesianToGrid(double x_coord, double y_coord) const;
        bool boundingBox(const GridCell &cell) const;
        float computeForwardObstacleDistance() const;

        // Control & Odometry (control.cu)
        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg);
        void oppOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg);
        CartesianState extractPose(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg) const;
        FrenetState globalToFrenet(const CartesianState &cart_state, int last_closest_idx) const;
        std::vector<F1TenthGrid::Reference> buildReferenceHorizon(float current_s, float safe_vx) const;
        DriveCommand computeDriveCommand(
            const std::vector<F1TenthGrid::Reference> &ref_horizon,
            float optimal_steer,
            float current_vel_cmd) const;

        // 4. Visualization (visualization.cu)
        void visualizeOccupancyGrid(const std_msgs::msg::Header &scan_header);
        void visualizeCenterline();
        void visualizeTrajectories(
            const F1TenthGrid::State &start_state,
            const std::vector<F1TenthGrid::Reference> &ref_horizon,
            float optimal_steer);
        std::pair<float, float> frenetToGlobal(float s, float d) const;
    };

} // namespace rmpc_f1tenth