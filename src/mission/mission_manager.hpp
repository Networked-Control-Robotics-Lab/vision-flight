#pragma once

#include "waypoint_mission.hpp"
#include "trajectory_mission.hpp"

class MissionManager {
	public:
	WaypointManager waypoint;
	TrajectoryManager trajectory;

	MissionManager() {};
	MissionManager(int target_id, int frame, bool traj_z_enabled): waypoint(target_id, frame),
                                                                       trajectory(target_id, traj_z_enabled) {}
	~MissionManager() {}
};
