#ifndef __MAVLINK_PARSER_HPP__
#define __MAVLINK_PARSER_HPP__

#include <stdint.h>
#include "mavlink.h"

#define CMD_LEN(list) (sizeof(list) / sizeof(struct mavlink_parser_item))
#define MAV_CMD_DEF(handler_function, id) \
        [handler_function ## _ID] = {id, handler_function}
#define ENUM_HANDLER_FUNC(handler_function) handler_function ## _ID

struct mavlink_parser_item {
	uint16_t msg_id;
	void (*handler)(mavlink_message_t *msg);
};

void parse_mavlink_received_msg(mavlink_message_t *msg);

#endif
