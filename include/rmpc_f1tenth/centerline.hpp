#pragma once 
#include <vector>
#include <string>
#include <tuple>

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

class Centerline {
public:
    Centerline() = default;

    // Loads CSV formatted: [x, y, yaw, curvature, left_bound, right_bound]
    bool loadFromCSV(const std::string& csv_path);

    // Returns the index of the closest waypoint to (px, py)
    size_t getClosestIndex(float px, float py, size_t prev_index = 0) const;

    // Dynamic spatial lookahead: extracts N horizon points scaled by forward velocity
    std::vector<Waypoint> getHorizon(float px, float py, float vx, 
                                     size_t horizon_steps, float dt) const;

    const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
    float getTotalLength() const { return total_length_; }
    bool isLoaded() const { return !waypoints_.empty(); }

private:
    std::vector<Waypoint> waypoints_;
    float total_length_{0.0f};
    void computeArcLengths();
};