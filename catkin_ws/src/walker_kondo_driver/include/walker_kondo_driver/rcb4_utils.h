#include "rcb4.h"

#define RCB4_SYS_RGST_ICS 0
//#define RCB4_SYS_RGST_LED 15
//#define SERVO_FREE_POSITION 0x8000
//#define SERVO_HOLD_POSITION 0x7FFF
//#define SERVO_NEUTRAL_POSITION 7500
#define RCB4_DOF 35

int set_system_register(KondoRef ki, int bit_num, char val);
int send_servo_vector(KondoRef ki, unsigned short av[], int len, int vel, int block);
int kondo_get_servos_three_separate(KondoRef ki, short int start_no, short int *data);
int kondo_get_servos_three(KondoRef ki, short int *data);
int kondo_read_analog_all(KondoRef ki, short int *result, int num);
