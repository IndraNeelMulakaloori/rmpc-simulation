
#pragma once
#include <math.h>
#include <array>

#include "../helper/randomized-mpc/examples/car_steering/problem.cuh"
#include "rmpc/problem.h"


inline constexpr float MIN_VX_FOR_SLIP = 0.5f;
inline constexpr int MAX_OBSTACLES = 8;
// Pajecka tire mdoel imported from the exisitng examples with Occupancy grid
namespace rmpc_f1tenth{

using car_steering::CarSteering;
struct Obstacle {
  float s;
  float d;
  float radius;
};
struct F1TenthGrid {
  using State = CarSteering::State;
  using Reference = CarSteering::Reference;

  struct Params {
    CarSteering::Params car;

    // Obstacle params from /helper/randomized-mpc/examples/car_obstacles/problem.cuh
    Obstacle obstacles[MAX_OBSTACLES];
    int n_obstacles{0};
    float car_radius{0.1016f};   // Half-width of F1Tenth car (~0.10m)
    float clearance{0.10f};    // Safety buffer around obstacle [m]
    float hit_cost{20000.0f};  // Flat collision penalty per step
    float track_length{0.0f};


    int8_t* cuda_occupancy_grid; // Ocuppancy grid which stores local obstacle map
    double resolution;          // e.g., 0.05
    int columns;               // e.g., 60
    int rows;                  // e.g., 60
    int x_offset;              
    int y_offset;
    double collision_penalty;   // e.g., 10000.0f

    float start_s;      // FOr lidar occupancy grid
    float start_d;
    float start_yaw_err;
    float start_k;
  };
  // Using Exsiitng step() from examples
  RMPC_HD static State step(const State& x, float steer, const Reference& ref,
                            const Params& p, float dt) {
    return CarSteering::step(x, steer, ref, p.car, dt);
  }
// Two-disc axle collision check from /helper/randomized-mpc/examples/car_obstacles/problem.cuh
  RMPC_HD static bool collides(const State& x, const Params& p) {
    const float cos = cosf(x.yaw);
    const float sin = sinf(x.yaw);
    const float axle[2] = {p.car.lf, -p.car.lr};

    for (int i = 0; i < p.n_obstacles; ++i) {
      const Obstacle& o = p.obstacles[i];
      const float reach = o.radius + p.car_radius + p.clearance;
      const float reach_sq = reach * reach;

      for (float a : axle) {
        float ds = x.s + a * cos - o.s;
        // Track periodic boundary wrap-around
        if (p.track_length > 0.0f && ds > 0.5f * p.track_length) {
          ds = p.track_length - ds;
        }
        const float dd = x.d + a * sin - o.d;
        if (ds * ds + dd * dd < reach_sq) {
          return true;
        }
      }
    }
    return false;
  }
  // O(1) Grid Obstacle look up.
  RMPC_HD static bool checkGrid(const State& x, const Params& p) {
    // float ds = x.s - p.start_s; 
    // float k = p.start_k;

    // // 1. Find the target point on the curved centerline
    // float theta = k * ds;
    // float x_c = (fabsf(k) > 1e-4f) ? (sinf(theta) / k) : ds;
    // float y_c = (fabsf(k) > 1e-4f) ? ((1.0f - cosf(theta)) / k) : 0.0f;

    // // 2. Shift to the absolute lateral offset (x.d)
    // float dx_fc = x_c - x.d * sinf(theta);
    // float dy_fc = y_c + x.d * cosf(theta);

    // // 3. Shift origin to the physical car's position (0, start_d)
    // float dx_car = dx_fc;
    // float dy_car = dy_fc - p.start_d;

    // // 4. Rotate by the car's yaw error to align with LIDAR X-Y axes
    // float cos_y = cosf(p.start_yaw_err);
    // float sin_y = sinf(p.start_yaw_err);
    
    // float x_cart = dx_car * cos_y + dy_car * sin_y;
    // float y_cart = -dx_car * sin_y + dy_car * cos_y;
    
    // int grid_x = static_cast<int>(x_cart / p.resolution) + p.x_offset;
    // int grid_y = static_cast<int>(y_cart / p.resolution) + p.y_offset;

    // if (grid_x < 0 || grid_x >= p.columns || grid_y < 0 || grid_y >= p.rows) return false;
    // int index = grid_x + (grid_y * p.columns);
    // return (p.cuda_occupancy_grid[index] > 50.0f);
  }

  RMPC_HD static float stageCost(const State& x, float steer,
                                 float steer_prev, const Reference& ref,
                                 const Params& p, int k, int n) {
                                     
    // base cost (cross-track error, steering effort, Pacejka slip)
    float base_cost = CarSteering::stageCost(x, steer, steer_prev, ref, p.car, k, n);
    
    // Penalising the trajecotry if ti hits teh obstacle
    float hit_cost = collides(x, p) ? (p.hit_cost / (float)n) : 0.0f;
    
    return base_cost + hit_cost;
  }

  RMPC_HD static float terminalCost(const State& x, const Reference& ref,
                                    const Params& p, int n) {
    return CarSteering::terminalCost(x, ref, p.car, n);
  }
};

static_assert(rmpc::Problem<F1TenthGrid>);


}  // namespace rmpc_f1tenth

