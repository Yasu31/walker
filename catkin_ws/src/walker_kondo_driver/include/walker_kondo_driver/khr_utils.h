#ifndef __KHR_UTILS_H__
#define __KHR_UTILS_H__

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
extern "C" {
#include "rcb4.h"
}
#define RCB4_SYS_RGST_ICS 0
#define RCB4_SYS_RGST_LED 15
#define SERVO_FREE_POSITION 0x8000
#define SERVO_HOLD_POSITION 0x7FFF
#define SERVO_NEWTRAL_POSITION 7500
#define KHR_DOF 22

void error(KondoRef ki);
std::string version_check();
int read_system_register();
int set_system_register(int bit_num, bool val);
int set_led_register(bool val);
int set_ics_switch(bool val);
int init_servo();
int set_serial_servo_register();
int set_serial_servo_register_free();
int copy_serial_servo_register_from_rom(unsigned short rom_addr, int servo_num);
int read_servo_register(char lower, char upper);
int register_servo_register_addr(unsigned short register_addr, int ics_num);
int copy_and_register_servo_register(unsigned short ram_addr, int servo_num);
int single_servo_action(unsigned char servo_id, unsigned short position, unsigned char speed);
int all_servo_action(unsigned short position[], unsigned char speed);
int windows_head_move();
unsigned int read_servo_position(unsigned char servo_id);
unsigned int read_servo_real_position(unsigned char servo_id);
int windows_lshoulderp_move();
int change_all_servo_gain(unsigned char gain);
int change_all_servo_gain(unsigned char gain_array[], int array_size);
double servo2angle(std::string name, unsigned short position);
unsigned short angle2servo(std::string name, double angle);

#endif  // __KHR_UTILS_H__
