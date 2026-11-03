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

        visualization_msgs::msg::Marker dashed_line;
        dashed_line.header.frame_id = "map";
        dashed_line.header.stamp = this->now();
        dashed_line.ns = "track_centerline";
        dashed_line.id = 0;
        dashed_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        dashed_line.action = visualization_msgs::msg::Marker::ADD;
        dashed_line.scale.x = 0.06; // Width of the dashed stripe
        
        // Bright yellow road-stripe color
        dashed_line.color.r = 1.0f;
        dashed_line.color.g = 0.85f;
        dashed_line.color.b = 0.0f;
        dashed_line.color.a = 0.95f;
        
        // Dash pattern: 0.4 m stripe, 0.3 m gap (0.7 m cycle)
        const float dash_length = 0.40f;
        const float gap_length = 0.30f;
        const float period = dash_length + gap_length;
        
        const size_t N = reference_path_.size();
        for (size_t i = 0; i < N; ++i) {
            size_t next_idx = (i + 1) % N;
            float s = reference_path_[i].s;
        
            // Only draw segment if within the solid part of the dash cycle
            if (std::fmod(s, period) < dash_length) {
                geometry_msgs::msg::Point p1, p2;
                p1.x = static_cast<double>(reference_path_[i].x);
                p1.y = static_cast<double>(reference_path_[i].y);
                p1.z = 0.02; // Placed just above ground
            
                p2.x = static_cast<double>(reference_path_[next_idx].x);
                p2.y = static_cast<double>(reference_path_[next_idx].y);
                p2.z = 0.02;
            
                dashed_line.points.push_back(p1);
                dashed_line.points.push_back(p2);
            }
        }
    
        centerline_publisher_->publish(dashed_line);
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
    const F1TenthGrid::State& start_state,
    const std::vector<F1TenthGrid::Reference>& ref_horizon,
    float optimal_steer) {

    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time current_time = this->now();

    // 1. Dense Candidate Rollout Fan
    visualization_msgs::msg::Marker candidate_lines;
    candidate_lines.header.frame_id = "map";
    candidate_lines.header.stamp = current_time;
    candidate_lines.ns = "candidate_trajectories";
    candidate_lines.id = 1;
    candidate_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    candidate_lines.action = visualization_msgs::msg::Marker::ADD;
    candidate_lines.scale.x = 0.008; // Crisp thin lines

    // Vibrant Electric Cyan with solid opacity to stand out on dark costmaps
    candidate_lines.color.r = 0.0f;
    candidate_lines.color.g = 0.85f;
    candidate_lines.color.b = 1.0f;
    candidate_lines.color.a = 0.45f;

    const int num_viz_trajs = 61; // Dense sampling envelope
    const float steer_min = -static_cast<float>(max_steering_limit_);
    const float steer_max = static_cast<float>(max_steering_limit_);
    const float steer_step = (steer_max - steer_min) / static_cast<float>(num_viz_trajs - 1);
    const float sim_vx = static_cast<float>(v_min_);

    for (int i = 0; i < num_viz_trajs; ++i) {
        float test_steer = steer_min + (static_cast<float>(i) * steer_step);
        float sim_s = start_state.s;
        float sim_d = start_state.d;

        for (int k = 0; k < horizon_ - 1; ++k) {
            auto [gx1, gy1] = frenetToGlobal(sim_s, sim_d);

            sim_s += sim_vx * static_cast<float>(dt_);
            sim_d += std::tan(test_steer) * sim_vx * static_cast<float>(dt_);

            auto [gx2, gy2] = frenetToGlobal(sim_s, sim_d);

            geometry_msgs::msg::Point p1, p2;
            p1.x = static_cast<double>(gx1);
            p1.y = static_cast<double>(gy1);
            p1.z = 0.04; // Sits completely above the occupancy grid

            p2.x = static_cast<double>(gx2);
            p2.y = static_cast<double>(gy2);
            p2.z = 0.04;

            candidate_lines.points.push_back(p1);
            candidate_lines.points.push_back(p2);
        }
    }

    // 2. Slimmer, Prominent Optimal Trajectory
    visualization_msgs::msg::Marker optimal_line;
    optimal_line.header.frame_id = "map";
    optimal_line.header.stamp = current_time;
    optimal_line.ns = "optimal_trajectory";
    optimal_line.id = 2;
    optimal_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    optimal_line.action = visualization_msgs::msg::Marker::ADD;
    optimal_line.scale.x = 0.018; // Slimmed down from 0.04

    // Solid bright crimson
    optimal_line.color.r = 1.0f;
    optimal_line.color.g = 0.05f;
    optimal_line.color.b = 0.05f;
    optimal_line.color.a = 1.0f;

    float opt_s = start_state.s;
    float opt_d = start_state.d;

    for (int k = 0; k < horizon_; ++k) {
        auto [gx, gy] = frenetToGlobal(opt_s, opt_d);
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(gx);
        p.y = static_cast<double>(gy);
        p.z = 0.06; // Highest elevation to avoid overlap
        optimal_line.points.push_back(p);

        opt_s += sim_vx * static_cast<float>(dt_);
        opt_d += std::tan(optimal_steer) * sim_vx * static_cast<float>(dt_);
    }

    marker_array.markers.push_back(candidate_lines);
    marker_array.markers.push_back(optimal_line);
    traj_publisher_->publish(marker_array);
}
} // namespace rmpc_f1tenth
