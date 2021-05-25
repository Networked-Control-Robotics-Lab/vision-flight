#include <vector>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.hpp"
#include "../mavlink/receiver.hpp"
#include "quadshell.hpp"
#include "waypoint_mission.hpp"
#include "trajectory_mission.hpp"

using namespace std;

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_exit(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	system("/bin/stty cooked echo");
	exit(0);
}

void shell_cmd_quit(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	system("/bin/stty cooked echo");
	exit(0);
}

void shell_cmd_mission(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	int uav_id = 1;
#if 0
	WaypointManager waypoint_manager(uav_id, WAYPOINT_CARTESIAN_FRAME);
	waypoint_manager.add(1, -1, 1);
	waypoint_manager.add(1, 1, 1);
	waypoint_manager.add(-1, 1, 1);
	waypoint_manager.add(-1, -1, 1);
	waypoint_manager.print_list();
	waypoint_manager.send();
#endif

	trajectory_t t;
	for(int i = 0; i < 8; i++) {
		t.pos_coeff[i] = i;
	}
	for(int i = 0; i < 7; i++) {
		t.vel_coeff[i] = i;
	}
	for(int i = 0; i < 6; i++) {
		t.accel_coeff[i] = i;
	}

	TrajectoryManager trajectory_manager(uav_id);
	trajectory_manager.add(t, t, t);
	trajectory_manager.add(t, t, t);
	trajectory_manager.add(t, t, t);
	trajectory_manager.add(t, t, t);
	trajectory_manager.print_list();
}
