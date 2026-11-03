#include "rmpc_f1tenth/rmpc_node.cuh"

namespace rmpc_f1tenth
{

    void RMPC::visualizeOccupancyGrid(const std_msgs::msg::Header &scan_header)
    {
        nav_msgs::msg::OccupancyGrid grid_msg;

        grid_msg.header.stamp = scan_header.stamp;
        grid_msg.header.frame_id = scan_header.frame_id;

        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = columns_;
        grid_msg.info.height = rows_;

        grid_msg.info.origin.position.x = -(x_offset_ * resolution_);
        grid_msg.info.origin.position.y = -(y_offset_ * resolution_);
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data = this->occupancy_grid_;
        grid_publisher_->publish(grid_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published local occupancy grid. Resolution: %.2f", this->resolution_);
    }

    void RMPC::visualizeCenterline()
    {
        if (reference_path_.empty())
            return;

        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "track_centerline";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.1; // Line width in meters

        // Cyan / White color for track reference line
        line_strip.color.r = 0.0f;
        line_strip.color.g = 0.8f;
        line_strip.color.b = 1.0f;
        line_strip.color.a = 0.8f;

        for (const auto &wp : reference_path_)
        {
            geometry_msgs::msg::Point p;
            p.x = static_cast<double>(wp.x);
            p.y = static_cast<double>(wp.y);
            p.z = 0.02; // Slightly above ground to prevent Z-fighting
            line_strip.points.push_back(p);
        }

        // Close the loop if it's a circuit
        if (!reference_path_.empty())
        {
            geometry_msgs::msg::Point p;
            p.x = static_cast<double>(reference_path_.front().x);
            p.y = static_cast<double>(reference_path_.front().y);
            p.z = 0.02;
            line_strip.points.push_back(p);
        }

        centerline_publisher_->publish(line_strip);
    }
    std::pair<float, float> RMPC::frenetToGlobal(float s, float d) const
    {
        if (reference_path_.empty())
            return {0.0f, 0.0f};

        const size_t num_wp = reference_path_.size();

        // Find the waypoint corresponding to arc-length s
        size_t idx = 0;
        while (idx < num_wp && reference_path_[idx].s <= s)
        {
            idx++;
        }
        idx = (idx > 0) ? idx - 1 : 0;
        size_t next_idx = (idx + 1) % num_wp;

        // Linear interpolation between waypoints for smooth transformation
        float s0 = reference_path_[idx].s;
        float s1 = reference_path_[next_idx].s;
        if (s1 < s0)
            s1 += reference_path_.back().s; // Handle wrap-around

    float t = (s - s0) / ((s1 - s0) > 1e-4f ? (s1 - s0) : 1.0f);
    t = std::clamp(t, 0.0f, 1.0f);

    const auto& wp0 = reference_path_[idx];
    const auto& wp1 = reference_path_[next_idx];

    // Interpolate position and heading
    float x_ref = wp0.x + t * (wp1.x - wp0.x);
    float y_ref = wp0.y + t * (wp1.y - wp0.y);
    float heading = wp0.heading + t * (wp1.heading - wp0.heading);

    // Offset by lateral error d along normal vector
    float global_x = x_ref - d * std::sin(heading);
    float global_y = y_ref + d * std::cos(heading);

    return {global_x, global_y};
    }

    void RMPC::visualizeTrajectories(
        const F1TenthGrid::State &start_state,
        const std::vector<F1TenthGrid::Reference> &ref_horizon,
        float optimal_steer)
    {

        visualization_msgs::msg::MarkerArray marker_array;
        rclcpp::Time current_time = this->now();

        // 1. Publish Candidate Rollout Fan (Thin Gray Lines)
        // Note: rmpc_ctrl_ exposes internal rollouts if available, or we publish
        // a representative visualization marker set for RViz feedback.
        visualization_msgs::msg::Marker candidate_lines;
        candidate_lines.header.frame_id = "map";
        candidate_lines.header.stamp = current_time;
        candidate_lines.ns = "candidate_trajectories";
        candidate_lines.id = 1;
        candidate_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
        candidate_lines.action = visualization_msgs::msg::Marker::ADD;
        candidate_lines.scale.x = 0.01; // Thin line width

        candidate_lines.color.r = 0.6f;
        candidate_lines.color.g = 0.6f;
        candidate_lines.color.b = 0.6f;
        candidate_lines.color.a = 0.25f; // Translucent gray

        // 2. Publish Optimal Selected Trajectory (Thick Bright Red Line)
        visualization_msgs::msg::Marker optimal_line;
        optimal_line.header.frame_id = "map";
        optimal_line.header.stamp = current_time;
        optimal_line.ns = "optimal_trajectory";
        optimal_line.id = 2;
        optimal_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        optimal_line.action = visualization_msgs::msg::Marker::ADD;
        optimal_line.scale.x = 0.04; // Thicker line width so it pops

        optimal_line.color.r = 1.0f;
        optimal_line.color.g = 0.0f;
        optimal_line.color.b = 0.0f;
        optimal_line.color.a = 1.0f; // Solid red

        // Reconstruct the optimal trajectory rollout using simple kinematic projection over horizon
        float sim_s = start_state.s;
        float sim_d = start_state.d;
        float sim_vx = static_cast<float>(v_min_); // Approximate progression velocity

        for (int k = 0; k < horizon_; ++k)
        {
            auto [gx, gy] = frenetToGlobal(sim_s, sim_d);
            geometry_msgs::msg::Point p;
            p.x = static_cast<double>(gx);
            p.y = static_cast<double>(gy);
            p.z = 0.05; // Slightly elevated over centerline to prevent Z-fighting
            optimal_line.points.push_back(p);

            // Advance simulation step
            sim_s += sim_vx * static_cast<float>(dt_);
            // Apply steering curvature influence to lateral offset d
            sim_d += std::tan(optimal_steer) * sim_vx * static_cast<float>(dt_);
        }

        marker_array.markers.push_back(candidate_lines);
        marker_array.markers.push_back(optimal_line);
        traj_publisher_->publish(marker_array);
    }
} // namespace rmpc_f1tenth
