#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/parser.hpp"
#include "../mavlink/receiver.hpp"

enum ENUM_MAV_CMDS {
	/* mavlink extended messages */
	ENUM_HANDLER_FUNC(mavlink_polynomial_trajectory_ack_handler),
	ENUM_HANDLER_FUNC(mavlink_polynomial_trajectory_position_debug_handler),
	ENUM_HANDLER_FUNC(mavlink_polynomial_trajectory_velocity_debug_handler),
	ENUM_HANDLER_FUNC(mavlink_polynomial_trajectory_acceleration_debug_handler)
};

struct mavlink_parser_item cmd_list[] = {
	/* mavlink extended messages */
	MAV_CMD_DEF(mavlink_polynomial_trajectory_ack_handler, 11002),
	MAV_CMD_DEF(mavlink_polynomial_trajectory_position_debug_handler, 11004),
	MAV_CMD_DEF(mavlink_polynomial_trajectory_velocity_debug_handler, 11005),
	MAV_CMD_DEF(mavlink_polynomial_trajectory_acceleration_debug_handler, 11006)
};

void parse_mavlink_received_msg(mavlink_message_t *msg)
{
	int i;
	for(i = 0; i < (signed int)CMD_LEN(cmd_list); i++) {
		if(msg->msgid == cmd_list[i].msg_id) {
			cmd_list[i].handler(msg);
			break;
		}
	}
}
