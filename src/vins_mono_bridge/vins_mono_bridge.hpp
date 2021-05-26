#pragma once

#include <string>
#include <thread>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"

using namespace std;

/* reception (imu datas) */
#define IMU_SERIAL_MSG_SIZE 27 
#define IMU_CHECKSUM_INIT_VAL 19

/* transmission (pose datas) */
#define VINS_MONO_CHECKSUM_INIT_VAL 19
#define VINS_MONO_SERIAL_MSG_SIZE 44

typedef struct {
	float accel[3];
	float gyro[3];

	int buf_pos;
	uint8_t buf[IMU_SERIAL_MSG_SIZE];
} imu_t ;

class VINSMonoBridge {
	private:
	int serial_fd;
	imu_t imu;
	bool kill_thread_signal;
	std::thread *thread_imu_rx;

	void serial_open(string serial_port, int baudrate);
	int serial_getc(char *c);
	void serial_puts(char *s, size_t size);
	uint8_t generate_checksum_byte(uint8_t init_val, uint8_t *payload, int payload_count);
	void send_pose_to_serial(float* pos, float* vel, float* q);
	void imu_msg_buf_push(uint8_t c);
	bool imu_message_decode(sensor_msgs::Imu& imu_msg);
	void serial_tx_thread_entry();

	public:
	VINSMonoBridge(string serial_port, int baudrate): kill_thread_signal(false),
                                                          thread_imu_rx(nullptr)
	{
		imu.buf_pos = 0;

		serial_open(serial_port, baudrate);
	}

	~VINSMonoBridge()
	{
		if(this->thread_imu_rx != nullptr) {
			kill_thread_signal = true;
			thread_imu_rx->join();
			delete thread_imu_rx;
		}
	}

	//copy constructor
	VINSMonoBridge(VINSMonoBridge const& rhs) = delete;

	//move constructor
	VINSMonoBridge(VINSMonoBridge&& rhs) = delete;

	//copy assignment
	VINSMonoBridge& operator=(VINSMonoBridge const& rhs) = delete;

	//move assignment
	VINSMonoBridge& operator=(VINSMonoBridge&& rhs) = delete;

	void launch_imu_message_listener();
};
