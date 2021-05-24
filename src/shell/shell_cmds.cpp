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
	WaypointManager waypoint_manager(1);
	waypoint_manager.create_rx_thread();
	waypoint_manager.add_local(1, -1, 1);
	waypoint_manager.add_local(1, 1, 1);
	waypoint_manager.add_local(-1, 1, 1);
	waypoint_manager.add_local(-1, -1, 1);
	waypoint_manager.print_list();
	waypoint_manager.send();
	waypoint_manager.stop_rx_thread();
}
