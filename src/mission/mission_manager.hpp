#pragma once

#include "waypoint_mission.hpp"
#include "trajectory_mission.hpp"

class MissionManager {
	public:
	WaypointManager waypoint;
	TrajectoryManager trajectory;

	MissionManager(int target_id, bool traj_z_enabled): waypoint(target_id, WAYPOINT_CARTESIAN_FRAME),
                                                            trajectory(target_id, traj_z_enabled) {}
	~MissionManager() {}
};
