#ifndef __MAVLINK_PUBLISHER_HPP__
#define __MAVLINL_PUBLISHER_HPP__

void send_mavlink_takeoff_cmd(void);
void send_mavlink_land_cmd(void);
void send_mavlink_position_target(float *pos_enu, float *vel_enu, float *acc_enu,
                                  float yaw, float yaw_rate);

void send_mavlink_polynomial_trajectory_start(bool loop);
void send_mavlink_polynomial_trajectory_stop();
void send_mavlink_polynomial_trajectory_write(uint8_t list_size, bool z_enabled, bool yaw_enabled);
void send_mavlink_polynomial_trajectory_item(uint8_t index, uint8_t type, float *traj_poly_coeff,
                float flight_time);

#endif
