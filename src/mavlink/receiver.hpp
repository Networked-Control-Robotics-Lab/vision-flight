#pragma once

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
}

void mavlink_attitude_quaternion_handler(mavlink_message_t *received_msg);
void mavlink_local_position_ned_handler(mavlink_message_t *received_msg);
void mavlink_polynomial_trajectory_ack_handler(mavlink_message_t *received_msg);
void mavlink_polynomial_trajectory_position_debug_handler(mavlink_message_t *received_msg);
void mavlink_polynomial_trajectory_velocity_debug_handler(mavlink_message_t *received_msg);
void mavlink_polynomial_trajectory_acceleration_debug_handler(mavlink_message_t *received_msg);

bool wait_mavlink_polynomial_trajectory_ack(uint8_t *ack_val);
