#include "system.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <iostream>
#include <math.h>

namespace rl
{

constexpr double kMaxVelocity{49.5};

void System::Cycle(const nlohmann::json &tele)
{
    Reset();

    // Paths from planner
    paths_ = {tele["previous_path_x"], tele["previous_path_y"], tele["end_path_s"], tele["end_path_d"]};
    sensor_fusion_ = tele["sensor_fusion"];

    UpdateEgoVehicle({tele["x"], tele["y"], tele["s"], tele["d"], tele["yaw"], tele["speed"]});

    CheckSurroundings();

    UpdateLaneDependingOnSituation();

    CreateTrajectory();
}

void System::CheckSurroundings()
{
    for (const auto &vehicle : sensor_fusion_)
    {
        if (IsVehicleInLane(vehicle, Lane::center))
        {
            UpdateSituationActualLane(vehicle);
        }
        else if (IsVehicleInLane(vehicle, Lane::left))
        {
            car_left_ |= UpdateSituationNextLane(vehicle);
        }
        else if (IsVehicleInLane(vehicle, Lane::right))
        {
            car_right_ |= UpdateSituationNextLane(vehicle);
        }
    }
}

bool System::IsVehicleInSameLane(const nlohmann::json &vehicle) const
{
    const float d = vehicle[6];
    return (d < (2U + 4U * current_lane_ + 2U) && d > (2U + 4U * current_lane_ - 2));
}

void System::UpdateSituationActualLane(const nlohmann::json &vehicle)
{
    const auto check_car_s = GetVehicleS(vehicle);

    const bool is_car_in_front{check_car_s > ego_vehicle_.s};
    const double current_distance{check_car_s - ego_vehicle_.s};

    if (is_car_in_front)
    {
        const double vx = vehicle[3];
        const double vy = vehicle[4];
        target_speed_ = sqrt(vx * vx + vy * vy);

        if (current_distance < 30U)
        {
            too_close_ = true;
        }
        else if (current_distance > 50U)
        {
            target_speed_ = kMaxVelocity;
        }
    }
}

bool System::UpdateSituationNextLane(const nlohmann::json &vehicle) const
{
    const auto check_car_s = GetVehicleS(vehicle);

    return (((check_car_s > ego_vehicle_.s) && ((check_car_s - ego_vehicle_.s) < 30U)) ||
            ((check_car_s < ego_vehicle_.s) && ((check_car_s - ego_vehicle_.s) > -7)));
}

bool System::IsVehicleInLane(const nlohmann::json &vehicle, System::Lane lane) const
{
    const float d = vehicle[6];

    return (d < ((2U + 4U * current_lane_ + 2U) + (static_cast<int>(lane) * 4)) &&
            d > ((2U + 4U * current_lane_ - 2) + (static_cast<int>(lane) * 4)));
}

double System::GetVehicleS(const nlohmann::json &vehicle) const
{
    const double vx = vehicle[3];
    const double vy = vehicle[4];
    const double check_speed = sqrt(vx * vx + vy * vy);
    double check_car_s = vehicle[5];

    check_car_s += ((double)paths_.previous_x.size() * .02 * check_speed);

    return check_car_s;
}

void System::UpdateEgoVehicle(const Vehicle vehicle)
{
    ego_vehicle_ = vehicle;
    const auto prev_size = paths_.previous_x.size();

    if (prev_size > 0)
    {
        ego_vehicle_.s = paths_.previous_s;
    }
}

void System::UpdateLaneDependingOnSituation()
{
    if (too_close_ && (!car_left_) && (current_lane_ > 0U))
    {
        --current_lane_;
    }
    else if (too_close_ && (!car_right_) && (current_lane_ < 2U))
    {
        ++current_lane_;
    }
}

Trajectory System::GetTrajectory() const { return trajectory_; }

void System::CreateTrajectory()
{
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    double reference_x = ego_vehicle_.x;
    double reference_y = ego_vehicle_.y;
    double reference_yaw = deg2rad(ego_vehicle_.yaw);

    const auto previous_size = paths_.previous_x.size();

    // Special case if only two points left
    if (previous_size < 2U)
    {
        ptsx.push_back(ego_vehicle_.x - cos(ego_vehicle_.yaw));
        ptsx.push_back(ego_vehicle_.x);

        ptsy.push_back(ego_vehicle_.y - sin(ego_vehicle_.yaw));
        ptsy.push_back(ego_vehicle_.y);
    }
    else
    {
        reference_x = paths_.previous_x[previous_size - 1];
        reference_y = paths_.previous_y[previous_size - 1];

        const double ref_x_prev = paths_.previous_x[previous_size - 2];
        const double ref_y_prev = paths_.previous_y[previous_size - 2];
        reference_yaw = atan2(reference_y - ref_y_prev, reference_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(reference_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(reference_y);
    }

    // create waypoints using also information from the lane
    const auto next_wp0 =
        getXY(ego_vehicle_.s + 30, (2 + 4 * current_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    const auto next_wp1 =
        getXY(ego_vehicle_.s + 60, (2 + 4 * current_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    const auto next_wp2 =
        getXY(ego_vehicle_.s + 90, (2 + 4 * current_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); ++i)
    {
        // 0 degree angle makes it easier
        const double shift_x = ptsx[i] - reference_x;
        const double shift_y = ptsy[i] - reference_y;

        ptsx[i] = (shift_x * cos(0 - reference_yaw) - shift_y * sin(0 - reference_yaw));
        ptsy[i] = (shift_x * sin(0 - reference_yaw) + shift_y * cos(0 - reference_yaw));
    }

    tk::spline s;

    s.set_points(ptsx, ptsy);

    // here we combine previous path with new path points
    for (int i = 0; i < previous_size; i++)
    {
        trajectory_.x.push_back(paths_.previous_x[i]);
        trajectory_.y.push_back(paths_.previous_y[i]);
    }

    GenerateVelocityDependendPoints(s, reference_x, reference_y, reference_yaw);
}

void System::GenerateVelocityDependendPoints(const tk::spline &s, const double reference_x, const double reference_y,
                                             const double reference_yaw)
{ // build up point from spline
    const double target_x = 30.0;
    const double target_y = s(target_x);
    const double target_distance = sqrt((target_x * target_x) + (target_y * target_y));

    double x_add_on = 0.0;

    const auto previous_size = paths_.previous_x.size();
    // Add the remaining points to the new path
    for (int i = 1; i < 50 - previous_size; i++)
    {

        if (too_close_ && velocity_ > target_speed_)
        {
            velocity_ -= .224;
        }
        else if (velocity_ < target_speed_ && velocity_ < kMaxVelocity)
        {
            velocity_ += .224;
        }

        double N = (target_distance / (.02 * velocity_ / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_cp = x_point;
        double y_cp = y_point;

        x_point = (x_cp * cos(reference_yaw) - y_cp * sin(reference_yaw));
        y_point = (x_cp * sin(reference_yaw) + y_cp * cos(reference_yaw));

        x_point += reference_x;
        y_point += reference_y;

        trajectory_.x.push_back(x_point);
        trajectory_.y.push_back(y_point);
    }
}

void System::Reset()
{
    too_close_ = false;
    target_speed_ = kMaxVelocity;
    car_left_ = false;
    car_right_ = false;
    trajectory_.x.clear();
    trajectory_.y.clear();
}
} // namespace rl
