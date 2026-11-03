#include "rmpc_f1tenth/rmpc_node.cuh"
namespace rmpc_f1tenth
{
    CartesianState RMPC::extractPose(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg) const
    {
        /**
         * @brief Extracts the pose information from the given odometry message.
         * @param {nav_msgs::msg::Odometry::ConstSharedPtr} pose_msg The incoming odometry message.
         * @return {CartesianState} The extracted Cartesian state.
         */
        CartesianState state;
        state.vx = pose_msg->twist.twist.linear.x;
        state.vy = pose_msg->twist.twist.linear.y;
        state.yaw_rate = pose_msg->twist.twist.angular.z;

        state.x = pose_msg->pose.pose.position.x;
        state.y = pose_msg->pose.pose.position.y;

        double q_x = pose_msg->pose.pose.orientation.x;
        double q_y = pose_msg->pose.pose.orientation.y;
        double q_z = pose_msg->pose.pose.orientation.z;
        double q_w = pose_msg->pose.pose.orientation.w;

        tf2::Quaternion quart(q_x, q_y, q_z, q_w);
        tf2::Matrix3x3 mat(quart);

        double roll, pitch;
        mat.getRPY(roll, pitch, state.yaw);

        return state;
    }
    FrenetState RMPC::globalToFrenet(const CartesianState &cart_state, int last_closest_idx) const
    {
        /**
         * @brief Converts the given Cartesian state to the Frenet frame.
         * @param {CartesianState} cart_state The input Cartesian state.
         * @param {int} last_closest_idx The index of the last closest waypoint.
         * @return {FrenetState} The corresponding Frenet state.
         */
        FrenetState frenet;
        if (reference_path_.empty())
        {
            return frenet;
        }

        // Gets the number of waypoints in the reference path &
        // initializes variables for finding the closest waypoint
        const int num_waypoints = static_cast<int>(reference_path_.size());
        int closest_idx = last_closest_idx;
        float min_dist_sq = 1e9f;
        float best_dx = 0.0f;
        float best_dy = 0.0f;

        // Isntead of searching the entire path,
        // we search a window around the last closest index
        const int search_back = 5;
        const int search_ahead = 30;

        // Iterating through the search window to find the closest waypoint
        for (int i = -search_back; i <= search_ahead; ++i)
        {
            // Modulo operation to wrap around the waypoint indices
            int idx = (last_closest_idx + i) % num_waypoints;
            if (idx < 0)
            {
                idx += num_waypoints;
            }
            // Compute the distance from the
            // current Cartesian state to the candidate waypoint
            // Avoiding sqrt for efficiency, we compare squared distances
            float dx = static_cast<float>(cart_state.x) - reference_path_[idx].x;
            float dy = static_cast<float>(cart_state.y) - reference_path_[idx].y;
            float dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_idx = idx;
                best_dx = dx;
                best_dy = dy;
            }
        }

        // Update the closest index for future calls
        frenet.closest_idx = closest_idx;
        const Waypoint &wp = reference_path_[closest_idx];

        // Transform the Cartesian coordinates
        // to the Frenet frame using the closest waypoint
        // The Frenet s-coordinate is the arc length along the reference path
        // The Frenet d-coordinate is the lateral offset from the reference path
        frenet.s = wp.s + (best_dx * std::cos(wp.heading) + best_dy * std::sin(wp.heading));
        frenet.d = best_dx * (-std::sin(wp.heading)) + best_dy * std::cos(wp.heading);
        frenet.curvature = wp.curvature;

        // Compute the yaw error in the Frenet frame, ensuring it is wrapped to [-pi, pi]
        float yaw_error = static_cast<float>(cart_state.yaw) - wp.heading;
        frenet.yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

        return frenet;
    }

    std::vector<F1TenthGrid::Reference> RMPC::buildReferenceHorizon(float current_s, float safe_vx) const
    {
        /**
         * @brief Builds a reference horizon for the RMPC controller based on the current s-coordinate and safe velocity.
         * @param {float} current_s The current s-coordinate along the reference path.
         * @param {float} safe_vx The safe longitudinal velocity of the vehicle.
         * @return {std::vector<F1TenthGrid::Reference>} A vector of reference points for the prediction horizon.
         */

        // Initialize the reference horizon with the specified number of steps
        std::vector<F1TenthGrid::Reference> reference_horizon(horizon_);
        const float track_length = reference_path_.back().s;
        const size_t num_wp = reference_path_.size();
        size_t search_idx = static_cast<size_t>(ego_closest_idx_);

        // Ensure the search index is within the valid range of waypoints
        for (int step = 0; step < horizon_; ++step)
        {
            // Compute the target s-coordinate for the current step
            // based on the current s and safe velocity
            float s_target = current_s + (static_cast<float>(step) * safe_vx * static_cast<float>(dt_));
            // Wrap around the s_target to ensure it stays within the track length
            s_target = std::fmod(s_target, track_length);
            if (s_target < 0.0f)
            {
                s_target += track_length;
            }

            // Advance sequentially along waypoints
            while (reference_path_[(search_idx + 1) % num_wp].s < s_target &&
                   reference_path_[(search_idx + 1) % num_wp].s > reference_path_[search_idx].s)
            {
                search_idx = (search_idx + 1) % num_wp;
            }
            // Get the waypoint corresponding to the current search index
            const auto &wp = reference_path_[search_idx];
            // Create a reference point based on the current waypoint
            F1TenthGrid::Reference ref;
            ref.d_target = 0.0f;
            ref.left_bound = wp.l_bound;
            ref.right_bound = -wp.r_bound;
            ref.curvature = wp.curvature;
            // Store the reference point in the reference horizon at the current step
            reference_horizon[step] = ref;
        }

        return reference_horizon;
    }

    DriveCommand RMPC::computeDriveCommand(
        const std::vector<F1TenthGrid::Reference> &ref_horizon,
        float optimal_steer,
        float current_vel_cmd) const
    {

        /** @brief Compute the drive command based on the reference horizon, optimal steering angle, and current velocity command.
         *  @param ref_horizon The reference horizon containing the target points.
         *  @param optimal_steer The optimal steering angle computed by the RMPC solver.
         *  @param current_vel_cmd The current velocity command.
         *  @return The computed drive command.
         */

        DriveCommand cmd;
        cmd.steering_angle = optimal_steer;

        // Look ahead for highest curvature in prediction horizon
        float max_k = 0.0f;
        for (int k = 0; k < horizon_; ++k)
        {
            max_k = std::max(max_k, std::abs(ref_horizon[k].curvature));
        }

        // Pacejka friction circle speed ceiling: v = sqrt(mu * g / kappa)
        float safe_corner_speed = std::sqrt((friction_coeff_ * gravity_) / (max_k + 1e-4f));

        // Bound speed against forward obstacles and limits
        // min_forward_dist_ is the distance to the nearest forward obstacle
        // The raw target velocity is the minimum of the safe corner speed
        // and the distance to the nearest forward obstacle
        double raw_target_velocity = std::min(
            static_cast<double>(safe_corner_speed),
            static_cast<double>(min_forward_dist_));

        raw_target_velocity = std::clamp(raw_target_velocity, v_min_, v_max_);

        // Slew rate limiter using max_accel and max_decel
        float dt_f = static_cast<float>(dt_);
        float accel_step = max_accel_ * dt_f;
        float decel_step = max_decel_ * dt_f;

        if (raw_target_velocity > current_vel_cmd)
        {
            cmd.speed = std::min(
                static_cast<float>(raw_target_velocity),
                current_vel_cmd + accel_step);
        }
        else
        {
            cmd.speed = std::max(
                static_cast<float>(raw_target_velocity),
                current_vel_cmd - decel_step);
        }

        return cmd;
    }

    void RMPC::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
    {
        /** @brief Callback function for handling odometry messages.
         *  @param pose_msg The received odometry message.
         */

        // Pose extraction, conversion from quaternion to euler angle
        CartesianState cart_state = extractPose(pose_msg);

        // Convert the coordiantes to frenet coorrdiantes
        FrenetState frenet_state = globalToFrenet(cart_state, ego_closest_idx_);
        ego_closest_idx_ = frenet_state.closest_idx;

        // Velocity floor to avoid Pacejka divide-by-zero singularities
        float safe_vx = static_cast<float>(std::max(v_min_, cart_state.vx));
        problem_params_.car.vx = safe_vx;
        problem_params_.track_length = reference_path_.back().s;

        // Bind local grid coordinate anchors for the CUDA kernel
        problem_params_.start_s = frenet_state.s;
        problem_params_.start_d = frenet_state.d;
        problem_params_.start_yaw_err = frenet_state.yaw_error;
        problem_params_.start_k = frenet_state.curvature;

        rmpc_ctrl_->setParams(problem_params_);
        // Construct the current state for RMPC
        F1TenthGrid::State current_state{
            frenet_state.s,
            frenet_state.d,
            frenet_state.yaw_error,
            static_cast<float>(cart_state.vy),
            static_cast<float>(cart_state.yaw_rate)};

        // Generate reference horizon along arc-length projection
        std::vector<F1TenthGrid::Reference> reference_horizon = buildReferenceHorizon(frenet_state.s, safe_vx);

        // Run GPU RMPC trajectory rollout optimization
        auto solve_result = rmpc_ctrl_->solve(current_state, current_steering_, reference_horizon);

        // Publish the drive command based on the RMPC solution
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "base_link";

        // If the RMPC solver fails, log an error and set the drive command to zero
        if (solve_result.isError())
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
            "RMPC Solver Failed: %s",
                solve_result.error());
            current_velocity_cmd_ = 0.0f;
            drive_msg.drive.speed = 0.0f;
            drive_msg.drive.steering_angle = 0.0f;
        }
        else
        {
            // If the RMPC solver succeeds,
            // Extract the optimal steering angle and compute the drive command
            float optimal_steer = solve_result.value().nextInput();
            current_steering_ = optimal_steer;

            // Slew rate and friction-governed velocity profile
            DriveCommand cmd = computeDriveCommand(reference_horizon, optimal_steer, current_velocity_cmd_);
            current_velocity_cmd_ = cmd.speed;

            drive_msg.drive.steering_angle = cmd.steering_angle;
            drive_msg.drive.speed = cmd.speed;
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "Drive Command: Speed = %.2f m/s, Steering Angle = %.2f rad",
                cmd.speed,
                cmd.steering_angle);
            // Trigger visualizer for candidate fan and optimal path
            visualizeTrajectories(current_state, reference_horizon, optimal_steer);
        }

        drive_publisher_->publish(drive_msg);
    }

    void RMPC::oppOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
    {
        /** @brief Callback function for handling opponent car odometry messages.
         *  @param pose_msg The received odometry message from the opponent car.
         */
        /**
         * @note: This callback is used when a candidate opponent car is active
         * We have migrated from static ground truth
         * to lidar based opponent detection
         * but this callback is still useful for testing and debugging purposes.
         */
        // Pose extraction, conversion from quaternion to euler angle
        // double x_opp, y_opp, yaw, vx, vy, yaw_rate;
        // extractPose(pose_msg, x_opp, y_opp, yaw, vx, vy, yaw_rate);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            //  "opp_x: %.2f, opp_y: %.2f, opp_yaw: %.2f, opp_vx : %.2f, opp_vy : %.2f, opp_yaw_rate : %.2f", x_opp, y_opp, vx, vy, yaw_rate);
        // Convert the coordiantes to frenet coorrdiantes
        // float s_opp, d_opp, yaw_error, kappa;
        // globalToFrenet(x_opp, y_opp, yaw, s_opp, d_opp, yaw_error, kappa, this->opp_closest_idx);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            //  "opp_s: %.2f, opp_d: %.2f, opp_yaw: %.2f, opp_yaw_err : %.2f, opp_curv : %.2f", s_opp, d_opp, yaw, yaw_error, kappa);

        //     // When opponent car is active:
        // this->problem_params.n_obstacles = 1;
        // this->problem_params.obstacles[0].s = s_opp;
        // this->problem_params.obstacles[0].d = d_opp;
        // // Exact bounding radius enclosing opponent car's wheelbase (0.3302m) and width (0.2032m)
        // this->problem_params.obstacles[0].radius = 0.3f;
    }

} // namespace rmpc_f1tenth