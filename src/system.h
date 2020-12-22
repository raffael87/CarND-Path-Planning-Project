#ifndef RL_SYSTEM_H
#define RL_SYSTEM_H

#include "json.hpp"
#include "spline.h"

#include <vector>

namespace rl
{

struct Vehicle
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

struct Paths
{
    std::vector<double> previous_x;
    std::vector<double> previous_y;
    double previous_s;
    double previous_d;
};

struct Trajectory
{
    std::vector<double> x;
    std::vector<double> y;
};

class System
{
  public:
    System(const std::vector<double> &map_waypoints_x, const std::vector<double> &map_waypoints_y,
           const std::vector<double> &map_waypoints_s, const std::vector<double> &map_waypoints_dx,
           const std::vector<double> &map_waypoints_dy)
        : map_waypoints_x_(map_waypoints_x), map_waypoints_y_(map_waypoints_y), map_waypoints_s_(map_waypoints_s),
          map_waypoints_dx_(map_waypoints_dx), map_waypoints_dy_(map_waypoints_dy)
    {
    }

    void Cycle(const nlohmann::json &tele);

    Trajectory GetTrajectory() const;

  private:
    enum class Lane
    {
        left = -1,
        center = 0,
        right = 1
    };

    void UpdateEgoVehicle(const Vehicle vehicle);
    bool IsVehicleInSameLane(const nlohmann::json &vehicle) const;
    bool IsVehicleInLane(const nlohmann::json &vehicle, Lane lane) const;
    void UpdateSituationActualLane(const nlohmann::json &vehicle);
    bool UpdateSituationNextLane(const nlohmann::json &vehicle) const;
    double GetVehicleS(const nlohmann::json &vehicle) const;
    void Reset();
    void UpdateLaneDependingOnSituation();
    void CreateTrajectory();
    void CheckSurroundings();
    void GenerateVelocityDependendPoints(const tk::spline &s, double ref_x, double ref_y, double ref_yaw);

    const std::vector<double> &map_waypoints_x_;
    const std::vector<double> &map_waypoints_y_;
    const std::vector<double> &map_waypoints_s_;
    const std::vector<double> &map_waypoints_dx_;
    const std::vector<double> &map_waypoints_dy_;

    Vehicle ego_vehicle_;
    Paths paths_;
    nlohmann::json sensor_fusion_;
    int current_lane_{1};
    bool too_close_{false};
    double target_speed_{49.5};
    double velocity_{0.0};
    bool car_left_{false};
    bool car_right_{false};
    Trajectory trajectory_;
};

} // namespace rl

#endif // RL_SYSTEM_H
