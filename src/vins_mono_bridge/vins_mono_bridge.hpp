#pragma once

#include <string>

using namespace std;

class VINSMonoBridge {
	private:
	int serial_fd;

	int serial_getc(char *c);
	void serial_puts(char *s, size_t size);
	uint8_t generate_vins_mono_checksum_byte(uint8_t *payload, int payload_count);
	void send_pose_to_serial(float* pos, float* vel, float* q);

	public:
	VINSMonoBridge(string serial_port, int baudrate);
	~VINSMonoBridge() {}
};
