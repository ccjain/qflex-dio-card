#ifndef APP_MODBUS_APP_H
#define APP_MODBUS_APP_H

#include <stdint.h>

void modbus_app_init(uint8_t slave_id);
void modbus_app_poll(void);

#endif
