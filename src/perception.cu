#include "rmpc_f1tenth/rmpc_node.cuh"

namespace rmpc_f1tenth
{

    /**
     * @brief perception.cu incldues the logic for LiDAR scan processing,
     * occupancy grid generation, and GPU memory management for RMPC controller.
     * It provides functions to convert LiDAR scan data into a 2D occupancy grid,
     * dilate the occupied cells to account for vehicle dimensions,
     * and transfer the occupancy grid to GPU memory for efficient access by the RMPC controller.
     */

    bool RMPC::boundingBox(const GridCell &cell) const
    {
        /**
         * @brief Checks if the given cell coordinates are within the boundaries of the grid.
         * @param {GridCell} cell  The grid cell to check.
         * @return {bool} True if the cell is within the grid boundaries, false otherwise.
         */
        return (cell.x_idx >= 0 && cell.x_idx < columns_ &&
                cell.y_idx >= 0 && cell.y_idx < rows_);
    }
    GridCell RMPC::convertCartesianToGrid(double x_coord, double y_coord) const
    {
        /**
         * @brief Converts Cartesian coordinates to grid cell indices.
         * @param {double} x_coord The x-coordinate in the Cartesian frame.
         * @param {double} y_coord The y-coordinate in the Cartesian frame.
         * @return {GridCell} The corresponding grid cell.
         */
        GridCell cell;
        cell.x_idx = static_cast<int>(x_coord / resolution_) + x_offset_;
        cell.y_idx = static_cast<int>(y_coord / resolution_) + y_offset_;
        return cell;
    }

    float RMPC::computeForwardObstacleDistance() const
    {
        /**
         * @brief Computes the minimum forward obstacle distance from the current position.
         * @return {float} The minimum forward obstacle distance in meters
         * if an obstacle is detected, otherwise returns 100.0f indicating a clear path.
         */
        const int y_center = y_offset_;
        const int x_start = x_offset_;
        const int x_end = columns_;

        for (int gx = x_start; gx < x_end; ++gx)
        {
            // lateral_clearance is set 1 as deliberate choice
            // more the clearance, more reactiveness but lose speed.
            // Its a trade off between speed and safety
            for (int gy = y_center - lateral_clearance_; gy <= y_center + lateral_clearance_; ++gy)
            {
                if (gy >= 0 && gy < rows_)
                {
                    int index = gx + (gy * columns_);
                    if (occupancy_grid_[index] == 100)
                    {
                        // Return distance in meters from car bumper
                        float distance = static_cast<float>(gx - x_start) * resolution_ ;

                        rclcpp::Clock steady_clock(RCL_STEADY_TIME); // Use steady clock for throttled logging

                        RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        steady_clock,
                        500,
                        "Obstacle detected at grid cell (%d, %d) with index %d. Distance: %.2f m",
                        gx, gy, index, distance);
                        return distance;
                    }
                }
            }
        }
        return 100.0f; // Clear path
    }

    void RMPC::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
    {
        /**
         * @brief Callback function for processing LiDAR scan messages.
         * @param {sensor_msgs::msg::LaserScan::ConstSharedPtr} scan_msg The incoming LiDAR scan message.
         */
        // Reset the Grid first
        std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), 0);

        // Rasterize and dilate LiDAR points
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float r = scan_msg->ranges[i];
            if (std::isinf(r) || std::isnan(r) || r > scan_msg->range_max || r < scan_msg->range_min)
            {
                continue;
            }

            float angle = scan_msg->angle_min + (static_cast<float>(i) * scan_msg->angle_increment);
            float x_cart = r * std::cos(angle);
            float y_cart = r * std::sin(angle);

            GridCell center_cell = convertCartesianToGrid(static_cast<double>(x_cart), static_cast<double>(y_cart));
            if (!boundingBox(center_cell))
            {
                continue;
            }

            // 2D Square inflation around hit cell
            for (int r_idx = -inflation_radius_; r_idx <= inflation_radius_; ++r_idx)
            {
                for (int c_idx = -inflation_radius_; c_idx <= inflation_radius_; ++c_idx)
                {

                    GridCell dilated_cell{center_cell.x_idx + r_idx, center_cell.y_idx + c_idx};
                    if (boundingBox(dilated_cell))
                    {
                        // 2d to 1d index conversion O(1) lookup for occupancy grid
                        int index = dilated_cell.x_idx + (dilated_cell.y_idx * columns_);
                        occupancy_grid_[index] = 100; // Mark as occupied (100 for Rviz visualization)
                    }
                }
            }
        }

        // Evaluate forward obstacle corridor on host grid
        min_forward_dist_ = computeForwardObstacleDistance();

        // RViz visualization
        visualizeOccupancyGrid(scan_msg->header);

        // Memory transfer to GPU
        cudaError_t err = cudaMemcpy(
            cuda_occupancy_grid_,
            occupancy_grid_.data(),
            allocation_size_,
            cudaMemcpyHostToDevice);
        // Fire Error message to terminal if cudaMemcpy fails
        if (err != cudaSuccess)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Failed cudaMemcpy to device: %s",
                cudaGetErrorString(err));
            return;
        }

        map_initialized_ = true;
    }
} // namespace rmpc_f1tenth