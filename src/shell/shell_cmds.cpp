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

static bool parse_float_from_str(char *str, float *value)
{
	char *end_ptr = NULL;
	errno = 0;
	*value = strtof(str, &end_ptr);
	if (errno != 0 || *end_ptr != '\0') {
		return false;
	} else {
		return true;
	}
}

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
	scan_best_camera_exposure(500, true);
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

void shell_cmd_fly(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char s[300] = {'\0'};

	if(param_cnt == 1) {
		shell_puts("fly x y z\n\r");
		return;
	}

	if(param_cnt != 4) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r");
		return;
	}

	float pos[3] = {0.0f};

	if (parse_float_from_str(param_list[1], &pos[0]) == false) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r");
		return;
	}

	if (parse_float_from_str(param_list[2], &pos[1]) == false) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r");
		return;
	}

	if (parse_float_from_str(param_list[3], &pos[2]) == false) {
		shell_puts("abort, bad arguments!\n\r"
		           "fly x y z\n\r");
		return;
	}

	sprintf(s, "enu waypoint (x, y, z) = (%fm, %fm, %fm)\n\r", pos[0], pos[1], pos[2]);
	shell_puts(s);

	struct shell_struct shell;
	shell_init_struct(&shell, "confirm fly command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_goto_cmd(0, pos[0], pos[1], pos[2]);

		//if() {
		//	shell_puts("failed, waypoint out of geo-fence!\n\r");
		//} else {
			shell_puts("command accept.\n\r");
		//}
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm takeoff command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_takeoff_cmd();
	} else {
		shell_puts("abort.\n\r");
	}
}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm landing command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_land_cmd();
	} else {
		shell_puts("abort.\n\r");
	}
}

static void waypoint_add_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	float pos[3] = {0.0f, 0.0f, 0.0f};
	float heading = 0.0f;
	float stay_time_sec = 1.0f;

	if(parse_float_from_str(param_list[2], &pos[0]) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;
	}
	if(parse_float_from_str(param_list[3], &pos[1]) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;
	}
	if(parse_float_from_str(param_list[4], &pos[2]) == false) {
		shell_puts("abort, bad arguments\n\r");
		return;

	}

	char s[200] = {'\0'};
	sprintf(s, "new waypoint: x=%.1fm, y=%.1fm, z=%.1fm\n\r", pos[0], pos[1], pos[2]);
	shell_puts(s);

	struct shell_struct shell;
	shell_init_struct(&shell, "confirm waypoint add command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.add(pos[0], pos[1], pos[2]);
		shell_puts("successfully added new waypoint.\n\r");
	} else {
		shell_puts("abort.\n\r");
	}
}

static void waypoint_halt_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm waypoint halt command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_halt_cmd();
	} else {
		shell_puts("abort.\n\r");
	}
}

static void waypoint_resume_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm waypoint resume command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_resume_cmd();
	} else {
		shell_puts("abort.\n\r");
	}
}

static void waypoint_start_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm waypoint start command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_start_cmd();
	} else {
		shell_puts("abort.\n\r");
	}
}

static void waypoint_send_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm waypoint send command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		mission_manager.waypoint.send_mission();
	} else {
		shell_puts("abort.\n\r");
	}
}

static void waypoint_list_cmd_handler(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX])
{
	if(mission_manager.waypoint.size() > 0) {
		mission_manager.waypoint.print_list();
	} else {
		shell_puts("error, waypoint list is empty.\n\r");
	}
}

void shell_cmd_waypoint(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 5) {
		if(strcmp(param_list[1], "add") == 0) {
			waypoint_add_cmd_handler(param_list);
		}
	} else if(param_cnt == 2) {
		if(strcmp(param_list[1], "halt") == 0) {
			waypoint_halt_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "resume") == 0) {
			waypoint_resume_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "start") == 0) {
			waypoint_start_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "send") == 0) {
			waypoint_send_cmd_handler(param_list);
		} else if(strcmp(param_list[1], "list") == 0) {
			waypoint_list_cmd_handler(param_list);
		} else {
			shell_puts("unknown waypoint command!\n\r");
		}
	} else {
		shell_puts("waypoint add x y z: add new waypoint\n\r"
                           "waypoint halt: halt waypoint mission\n\r"
                           "waypoint resume: resume halted waypoint mission\n\r"
                           "waypoint start: start waypoint mission\n\r"
                           "waypoint send: send waypoint mission to the uav\n\r"
                           "waypoint list: print waypoint mission\n\r");
	}
}

void shell_cmd_traj(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
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
