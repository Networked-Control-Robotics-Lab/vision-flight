#include "serial.hpp"

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

static void send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts((char *)buf, len);
}

void send_mavlink_takeoff_cmd(void)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_NAV_TAKEOFF;
	uint8_t confirm = 0;
	float params[7] = {0};

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_land_cmd(void)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_NAV_LAND;
	uint8_t confirm = 0;
	float params[7];

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_polynomial_trajectory_start(bool loop)
{
	bool altitude_fixed = true;

	uint8_t target_system = 0;
	uint8_t target_component = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_cmd_pack_chan(0, 1, MAVLINK_COMM_1, &msg,
	                target_system, target_component,
	                TRAJECTORY_FOLLOWING_START, altitude_fixed);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_polynomial_trajectory_stop()
{
	uint8_t target_system = 0;
	uint8_t target_component = 0;
	uint8_t option = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_cmd_pack_chan(0, 1, MAVLINK_COMM_1, &msg,
	                target_system, target_component,
	                TRAJECTORY_FOLLOWING_STOP, option);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_polynomial_trajectory_write(uint8_t list_size, bool z_enabled, bool yaw_enabled)
{
	uint8_t _z_enabled = (z_enabled == true ? 1 : 0);
	uint8_t _yaw_enabled = (yaw_enabled == true ? 1 : 0);

	uint8_t target_system = 0;
	uint8_t target_component = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_write_pack_chan(0, 1, MAVLINK_COMM_1, &msg, target_system,
	                target_component, list_size,
	                _z_enabled, _yaw_enabled);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_polynomial_trajectory_item(uint8_t index, uint8_t type, float *traj_poly_coeff,
                float flight_time)
{
	uint8_t target_system = 0;
	uint8_t target_component = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_item_pack_chan(0, 1, MAVLINK_COMM_1, &msg,
	                target_system, target_component,
	                type, index, traj_poly_coeff,
	                flight_time);
	send_mavlink_msg_to_serial(&msg);
}
