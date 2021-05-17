#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <unistd.h>

void serial_init(const char *port_name, int baudrate);
void serial_puts(char *s, size_t size);
int serial_getc(char *c);

#endif
