#include <iostream>
#include "serial.hpp"
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
#include "../mavlink/publisher.hpp"
#include "../mavlink/parser.hpp"

mavlink_message_t mavlink_recpt_msg;
mavlink_status_t mavlink_recpt_status;

uint8_t received_mavlink_msg;

void mavlink_thread_entry(void)
{
	char c;
	while(1) {
		if(serial_getc(&c) != -1) {
			//std::cout << c;
			received_mavlink_msg =
			        mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c,
			                           &mavlink_recpt_msg, &mavlink_recpt_status);
		}

		/* parse incoming mavlink message and call the message handler */
		if(received_mavlink_msg == 1) {
			parse_mavlink_received_msg(&mavlink_recpt_msg);
			received_mavlink_msg = 0;
		}
	}
}
