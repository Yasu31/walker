#include "rcb4_utils.h"

int printhex(KondoRef ki, int len)
{
  int j;
  printf(" %d bytes: ", len);
  for (j = 0; j < len; j++)
    printf("%x ", ki->swap[j]);
  printf("\n");
}

int send_servo_vector(KondoRef ki, unsigned short av[], int len, int vel, int block)
{
  int i,size;

  size =  8+ 2*len + 1;
  ki->swap[0] = size; // size 0x35 : 8+22*2+1=8+44+1=53= 16*3 + 5
  ki->swap[1] = 0x10;  // multi servo motion
  // for (i=2; i<7; i++) ki->swap[i] = 0xff;  //
  ki->swap[2] = 0xff;  ki->swap[3] = 0xff;  ki->swap[4] = 0xff;
  ki->swap[5] = 0xff;  ki->swap[6] = 0xff;
  ki->swap[7] = 0xff - (vel>0xff?0xff:(vel<0?0:(0xff & vel)));  // velocity
  for (i=0; i<len; i++) {
    ki->swap[i*2+8] = (UCHAR) (av[i] & 0xff);
    ki->swap[i*2+9] = (UCHAR) ((av[i]>>8) & 0xff);
  }
  ki->swap[size-1] = kondo_checksum(ki, size-1);

  return kondo_trx(ki, size, 4);
  //return printhex(ki, 53);
}

// RCB4のシステムレジスタ（RAMのアドレス$0000hから始まる16bit）を読み取る。
// 0-7bitはki->swap[3]に、8-15bitはki->swap[4]に格納される。
// return <  0 : エラー
//        == 5 : 成功
int read_system_register(KondoRef ki) {
  ki->swap[0] = 10;
  ki->swap[1] = RCB4_CMD_MOV;
  ki->swap[2] = RCB4_RAM_TO_COM;
  ki->swap[3] = 0x00;
  ki->swap[4] = 0x00;
  ki->swap[5] = 0x00;
  ki->swap[6] = 0x00;
  ki->swap[7] = 0x00;
  ki->swap[8] = 2;
  ki->swap[9] = kondo_checksum(ki, 9);
  return kondo_trx(ki, 10, 5);
}

// RCB4のRAMのシステムレジスタの指定したビットに値を書き込む。他のビットはそのまま。
// bit_num: 書き込みたいビット番号 (0~15)
// val:     書き込みたい値
// return <  0 : エラー
//        == 4 : 成功
//        other: バグ
int set_system_register(KondoRef ki, int bit_num, char val) {
  int ret;
  if (bit_num < 0 || bit_num > 15) {
    //    ROS_WARN("[set_system_register] bit_num is out of range.");
    return -1;
  }
  char val_byte = val ?
      0x01 << (bit_num % 8) : // example: bit_num=2 -> val_byte = 0x00000100
    ~(0x01 << (bit_num % 8)); // example: bit_num=2 -> val_byte = 0x11111011
  ret = read_system_register(ki);
  if (ret<0) return ret;
  char crt_register[2] = {ki->swap[2], ki->swap[3]};
  ki->swap[0] = 9;
  ki->swap[1] = RCB4_CMD_MOV;
  ki->swap[2] = RCB4_COM_TO_RAM;
  ki->swap[3] = 0x00;
  ki->swap[4] = 0x00;
  ki->swap[5] = 0x00;
  if (bit_num < 8) {
    ki->swap[6] = val ?
      (crt_register[0] | val_byte) : (crt_register[0] & val_byte);
    ki->swap[7] = crt_register[1];
  } else {
    ki->swap[6] = crt_register[0];
    ki->swap[7] = val ?
      (crt_register[1] | val_byte) : (crt_register[1] & val_byte);
  }
  ki->swap[8] = kondo_checksum(ki, 8);
  return kondo_trx(ki, 9, 4);
}

// windowsから一度ROMに書き込んでおくと設定がコピーできる。
// idが同じでポートが違うサーボを一つだけ動かすこともこれで可能になる。
int copy_serial_servo_register_from_rom(KondoRef ki, unsigned short ram_addr, int servo_num) {
  unsigned long rom_addr = 0x00626 + (servo_num * 20);
  // ROS_INFO("rom_addr = %x", rom_addr);
  ki->swap[0]  = 11;                               // data size
  ki->swap[1]  = 0x00;                             // MOV
  ki->swap[2]  = 0x03;                             // ROM -> RAM
  ki->swap[3]  = (unsigned char)(ram_addr >> 0);   // RAM addr: lower byte
  ki->swap[4]  = (unsigned char)(ram_addr >> 8);   //           upper byte
  ki->swap[5]  = 0x00;                             // (fixed)
  ki->swap[6]  = (unsigned char)(rom_addr >>  0);  // ROM addr  0- 7 bit
  ki->swap[7]  = (unsigned char)(rom_addr >>  8);  //           8-15 bit
  ki->swap[8]  = (unsigned char)(rom_addr >> 16);  //          16-23 bit
  ki->swap[9]  = 20;                               // reply data size
  ki->swap[10] = kondo_checksum(ki, 10);          // checksum
  return kondo_trx(ki, 11, 4);
}

int register_servo_register_addr(KondoRef ki, unsigned short register_addr, int ics_num) {
  unsigned short ics_addr = 0x0044 + (2 * ics_num);
  ki->swap[0] = 9;
  ki->swap[1] = RCB4_CMD_MOV;
  ki->swap[2] = RCB4_COM_TO_RAM;
  ki->swap[3] = (unsigned char)(ics_addr >> 0);
  ki->swap[4] = (unsigned char)(ics_addr >> 8);
  ki->swap[5] = 0x00;
  ki->swap[6] = (unsigned char)(register_addr >> 0);
  ki->swap[7] = (unsigned char)(register_addr >> 8);
  ki->swap[8] = kondo_checksum(ki, 8);
  return kondo_trx(ki, 9, 4);
}

int copy_and_register_servo_register(KondoRef ki, unsigned short ram_addr, int servo_num) {
  copy_serial_servo_register_from_rom(ki, ram_addr, servo_num);
  return register_servo_register_addr(ki, ram_addr, servo_num);
}

int kondo_get_servos_byte(KondoRef ki, UINT servo_idx, UINT offset,
		     UINT bytes, UCHAR *data)
{
	assert(ki);
	int i;

	// check port number range
	if (servo_idx < 0 || servo_idx > RCB4_NUM_SERVOS)
		kondo_error(ki, "Invalid servo index");

	// check field
	if (offset < 0 || offset >= 20)
		kondo_error(ki, "Invalid servo field");

	UINT ram_addr = RCB4_ADDR_SERVO + (RCB4_SERVO_DATA_SIZE * servo_idx)
			+ offset;

	// command
	ki->swap[0] = 10; // number of bytes
	ki->swap[1] = RCB4_CMD_MOV; // move command
	ki->swap[2] = RCB4_RAM_TO_COM; // RAM to COM
	ki->swap[3] = 0; // addr L
	ki->swap[4] = 0; // addr M
	ki->swap[5] = 0; // addr H
	ki->swap[6] = (UCHAR) (ram_addr); // ram L
	ki->swap[7] = (UCHAR) (ram_addr >> 8); // ram M
	ki->swap[8] = bytes; // bytes to move
	ki->swap[9] = kondo_checksum(ki, 9); // checksum

	// send 10, expect 5 in response
	if ((i = kondo_trx(ki, 10, 3+bytes)) < 0) {
	  printf("error in servos\n");
	  return i;
	}
	// verify response
	if (i != (3+bytes) || ki->swap[1] != RCB4_CMD_MOV) {
	  printf("i=%d bytes=%d ki->swap[1]=%d\n", i, bytes, ki->swap[1]);
	  //kondo_error(ki, "Response was not valid");
	}

	// everything ok, return result
	for (i=0; i< bytes; i++) data[i] = (UCHAR) ki->swap[2+i];

	return bytes;
}

int kondo_get_servos_word(KondoRef ki, UINT servo_idx, UINT offset,
		     UINT bytes, short int *data)
{
	assert(ki);
	int i;

	// check port number range
	if (servo_idx < 0 || servo_idx > RCB4_NUM_SERVOS)
		kondo_error(ki, "Invalid servo index");

	// check field
	if (offset < 0 || offset >= 58)
		kondo_error(ki, "Invalid servo field");

	UINT ram_addr = RCB4_ADDR_SERVO + (RCB4_SERVO_DATA_SIZE * servo_idx)
			+ offset;

	// command
	ki->swap[0] = 10; // number of bytes
	ki->swap[1] = RCB4_CMD_MOV; // move command
	ki->swap[2] = RCB4_RAM_TO_COM; // RAM to COM
	ki->swap[3] = 0; // addr L
	ki->swap[4] = 0; // addr M
	ki->swap[5] = 0; // addr H
	ki->swap[6] = (UCHAR) (ram_addr); // ram L
	ki->swap[7] = (UCHAR) (ram_addr >> 8); // ram M
	ki->swap[8] = bytes; // bytes to move
	ki->swap[9] = kondo_checksum(ki, 9); // checksum

	// send 10, expect 5 in response
	if ((i = kondo_trx(ki, 10, 3+bytes)) < 0) {
	  printf("error in servos\n");
	  return i;
	}
	// verify response
	if (i != (3+bytes) || ki->swap[1] != RCB4_CMD_MOV) {
	  printf("i=%d bytes=%d ki->swap[1]=%d\n", i, bytes, ki->swap[1]);
	  //kondo_error(ki, "Response was not valid");
	}

	// everything ok, return result
	for (i=0; i< (bytes/2); i++)
	  data[i] = (int) ((ki->swap[3+i*2] << 8) | ki->swap[2+i*2]);

	return bytes;
}

int kondo_get_servos_all_word(KondoRef ki, short int *data)
{
  register int i=0,j=0,vi,res;
  short int seg[6*10];
  vi=0;
  for (i=0; i<=RCB4_DOF/6; i++) {
    res = kondo_get_servos_word((KondoRef)ki,i*6, 0, 6*20, seg);
    for (j=0; j<6*10; j++) {
      if (vi>=RCB4_DOF*10) return vi;
      data[vi++]= seg[j];
    }
  }
  return vi;
}  

int kondo_get_servos_all_byte(KondoRef ki, UCHAR *data)
{
  register int i=0,j=0,vi,res;
  UCHAR seg[6*20];
  vi=0;
  for (i=0; i<=RCB4_DOF/6; i++) {
    res = kondo_get_servos_byte((KondoRef)ki,i*6, 0, 6*20, seg);
    for (j=0; j<6*20; j++) {
      if (vi>=RCB4_DOF*20) return vi;
      data[vi++]= seg[j];
    }
  }
  return vi;
}  

int kondo_get_servos_three_separate(KondoRef ki, short int start_no, short int *data)
{
  register int vi,res;
  short int seg[63];
  vi=3*7*start_no;
  res = kondo_get_servos_word((KondoRef)ki,start_no*7, 2, 126, seg);
  for (int i=0; i<7; i++) {
    if (vi>=RCB4_DOF*3) return vi;
    data[vi++]= seg[i*10];
    data[vi++]= seg[i*10+1];
    data[vi++]= seg[i*10+2];
  }
  return vi;
}

int kondo_get_servos_three(KondoRef ki, short int *data)
{
  register int i=0,j=0,vi,res;
  short int seg[63];
  vi=0;
  for (i=0; i<RCB4_DOF/7; i++) {
    res = kondo_get_servos_word((KondoRef)ki,i*7, 2, 126, seg);
    for (j=0; j<7; j++) {
      if (vi>=RCB4_DOF*3) return vi;
      data[vi++]= seg[j*10];
      data[vi++]= seg[j*10+1];
      data[vi++]= seg[j*10+2];
    }
  }
  return vi;
}  

int kondo_read_analog_all(KondoRef ki, short int *result, int num)
{
	assert(ki);
	register int i,mem_l;

	mem_l = RCB4_ADDR_AD_REF_BASE;

	// build command to read mem_h from RAM to COM
	ki->swap[0] = 10; // number of bytes
	ki->swap[1] = RCB4_CMD_MOV; // move command
	ki->swap[2] = RCB4_RAM_TO_COM; // RAM to COM
	ki->swap[3] = 0; // dest addr L (0 for COM)
	ki->swap[4] = 0; // dest addr M (0 for COM)
	ki->swap[5] = 0; // dest addr H (0 for COM)
	ki->swap[6] = (UCHAR) (mem_l); // mem_h low byte
	ki->swap[7] = (UCHAR) (mem_l >> 8); // mem_h high byte
	ki->swap[8] = 2*11 + 2*num; // bytes to move
	ki->swap[9] = kondo_checksum(ki, 9); // checksum

	// send 10, expect 5 in response
	if ((i = kondo_trx(ki, 10, 3 + 2*11 + 2*num)) < 0) {
	  printf("i=%d\n", i);
	  return i;
	}
	// verify response
	if ((i != 3+2*11+2*num) ||( ki->swap[1] != RCB4_CMD_MOV)) {
	  //kondo_error(ki, "Response was not analog values");
	  printf("i=%d, ki->swap[1]=%d\n", i, ki->swap[1]);
	}

	for (i=0; i<num; i++) {
	  result[i]= (short int)
	    ((((ki->swap[3+i*2]) << 8) | ki->swap[2+i*2])
	     +
	     (((ki->swap[3+(i+11)*2]) << 8) | ki->swap[2+(i+11)*2]));
	}
	return num;
}
