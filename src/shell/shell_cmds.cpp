#include <vector>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "quadshell.hpp"
#include "waypoint_mission.hpp"
#include "trajectory_mission.hpp"
#include "mission_manager.hpp"
#include "exposure_ctrl.hpp"
#include "arducam_ros_ctrl.hpp"

using namespace std;

extern MissionManager mission_manager;

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_cls();
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

void shell_cmd_exposure(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	scan_best_camera_exposure(5000, true);
}

void shell_cmd_camera(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)

{
	if(param_cnt == 2) {
		if(strcmp(param_list[1], "auto") == 0) {
			printf("arducam: auto mode\n\r");
			arducam_ros_auto_mode();
		} else if(strcmp(param_list[1], "trigger") == 0) {
			printf("arducam: trigger mode\n\r");
			arducam_ros_trigger_mode();
		}
	}
}

void shell_cmd_waypoint(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 5) {
		if(strcmp(param_list[1], "add") == 0) {
			//waypoint_add_cmd_handler(param_list);
		}
	} else if(param_cnt == 2) {
		if(strcmp(param_list[1], "start") == 0) {
			//waypoint_start_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "list") == 0) {
			//waypoint_list_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "halt") == 0) {
			//waypoint_halt_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "resume") == 0) {
			//waypoint_resume_cmd_handler(param_list);
		} else {
			shell_puts("unknown waypoint command!\n\r");
		}
	} else {
		shell_puts("waypoint add x y z: add new waypoint\n\r"
		           "waypoint start: start waypoint waypoint\n\r"
		           "waypoint list: list current waypoint list\n\r"
		           "waypoint halt: halt current executing waypoint waypoint\n\r"
		           "waypoint resume: resume current halting waypoint waypoint\n\r");
	}
}

void shell_cmd_traj(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 2) {
		if(strcmp(param_list[1], "plan") == 0) {
			//shell_cmd_traj_plan(param_list, param_cnt);
		} else if(strcmp(param_list[1], "start") == 0) {
			//shell_cmd_traj_start(param_list, param_cnt);
		} else if(strcmp(param_list[1], "stop") == 0) {
			//shell_cmd_traj_stop(param_list, param_cnt);
		}

	} else {
		printf("traj plan: plan trajectory\n\r"
		       "traj start: start trajectoy following\n\r"
		       "traj stop: stop trajectory following\n\r");
	}
}

void shell_cmd_mission(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	mission_manager.waypoint.add(1, -1, 1);
	mission_manager.waypoint.add(1, 1, 1);
	mission_manager.waypoint.add(-1, 1, 1);
	mission_manager.waypoint.add(-1, -1, 1);
	mission_manager.waypoint.print_list();
	mission_manager.waypoint.send_mission();

	mission_manager.waypoint.send_takeoff_cmd();
	mission_manager.waypoint.send_land_cmd();
	mission_manager.waypoint.send_halt_cmd();
	mission_manager.waypoint.send_resume_cmd();
	mission_manager.waypoint.send_start_cmd();

	trajectory_t t;
	for(int i = 0; i < 8; i++) {
		t.coeff[i] = i;
	}
	float flight_time = 2;

	mission_manager.trajectory.add(t, t, t, flight_time);
	mission_manager.trajectory.add(t, t, t, flight_time);
	mission_manager.trajectory.add(t, t, t, flight_time);
	mission_manager.trajectory.add(t, t, t, flight_time);
	mission_manager.trajectory.print_list();
	mission_manager.trajectory.send_mission();
}
