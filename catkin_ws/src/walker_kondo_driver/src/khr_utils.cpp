#include "khr_utils.h"

KondoInstance ki;

void error(KondoRef ki) {
  if(ki) {
    printf("%s", ki->error);
    kondo_close(ki);
  }
  exit(-1);
}

// RCB4のバージョンをチェックする
// return: RCB4から返ってきたバージョン情報
std::string version_check() {
  ki.swap[0] = 0x03;
  ki.swap[1] = 0xfd;
  ki.swap[2] = kondo_checksum(&ki, 2);
  if (kondo_trx(&ki, 3, 35) < 0) {
    ROS_INFO("kondo_trx error.");
  }
  char version[33];
  std::string version_str;
  memcpy(version, &ki.swap[2], 32);
  version[32] = '\0';
  version_str = version;
  return version_str;
}

// RCB4のシステムレジスタ（RAMのアドレス$0000hから始まる16bit）を読み取る。
// 0-7bitはki.swap[3]に、8-15bitはki.swap[4]に格納される。
// return <  0 : エラー
//        == 5 : 成功
int read_system_register() {
  ki.swap[0] = 10;
  ki.swap[1] = RCB4_CMD_MOV;
  ki.swap[2] = RCB4_RAM_TO_COM;
  ki.swap[3] = 0x00;
  ki.swap[4] = 0x00;
  ki.swap[5] = 0x00;
  ki.swap[6] = 0x00;
  ki.swap[7] = 0x00;
  ki.swap[8] = 2;
  ki.swap[9] = kondo_checksum(&ki, 9);
  return kondo_trx(&ki, 10, 5);
}

// RCB4のRAMのシステムレジスタの指定したビットに値を書き込む。他のビットはそのまま。
// bit_num: 書き込みたいビット番号 (0~15)
// val:     書き込みたい値
// return <  0 : エラー
//        == 4 : 成功
//        other: バグ
int set_system_register(int bit_num, bool val) {
  if (bit_num < 0 || bit_num > 15) {
    ROS_WARN("[set_system_register] bit_num is out of range.");
    return -1;
  }
  char val_byte = val ?
      0x01 << (bit_num % 8) : // example: bit_num=2 -> val_byte = 0x00000100
    ~(0x01 << (bit_num % 8)); // example: bit_num=2 -> val_byte = 0x11111011
  read_system_register();
  char crt_register[2] = {ki.swap[2], ki.swap[3]};
  ki.swap[0] = 9;
  ki.swap[1] = RCB4_CMD_MOV;
  ki.swap[2] = RCB4_COM_TO_RAM;
  ki.swap[3] = 0x00;
  ki.swap[4] = 0x00;
  ki.swap[5] = 0x00;
  if (bit_num < 8) {
    ki.swap[6] = val ? (crt_register[0] | val_byte) : (crt_register[0] & val_byte);
    ki.swap[7] = crt_register[1];
  } else {
    ki.swap[6] = crt_register[0];
    ki.swap[7] = val ? (crt_register[1] | val_byte) : (crt_register[1] & val_byte);
  }
  ki.swap[8] = kondo_checksum(&ki, 8);
  return kondo_trx(&ki, 9, 4);
}

int set_led_register(bool val) {
  return set_system_register(RCB4_SYS_RGST_LED, val);
}

int set_ics_switch(bool val) {
  return set_system_register(RCB4_SYS_RGST_ICS, val);
}

int init_servo() {
  unsigned char servo_ids[KHR_DOF]={4,5,6,7,8,9,10,11,19,30};
  // set serial servo register
  for (int servo_num = 0; servo_num < KHR_DOF; servo_num++ ) {
    unsigned short ram_addr = 0x0090 + (0x0014 * servo_num);
    copy_and_register_servo_register(ram_addr,  servo_ids[servo_num]);
  }

  // do not run ROM program
  set_system_register(1,0);
  // set baud rate of communication with servos to 1.25Mbps
  set_system_register(13,0);
  set_system_register(14,1);

  // set baud rate of communication with RCB to 1.25Mbps
  set_system_register(4,0);
  set_system_register(5,1);

  // set communication interval to 10ms
  set_system_register(4,0);
  set_system_register(5,0);
  // set ics switch on
  return set_ics_switch(true);
}

int set_serial_servo_register(unsigned short ram_addr, unsigned char servo_id) {
  ki.swap[0] = 27;
  ki.swap[1] = RCB4_CMD_MOV;
  ki.swap[2] = RCB4_COM_TO_RAM;
  ki.swap[3] = (unsigned char)(ram_addr>>0);
  ki.swap[4] = (unsigned char)(ram_addr>>8);
  ki.swap[5] = 0x00;
  // 内容
  ki.swap[6]  = 0x00;  // +0  レジスタ配置の内容種別
  ki.swap[7]  = servo_id;  // +1  シリアルサーボのID
  ki.swap[8]  = 0x00;  // +2  基準値
  ki.swap[9]  = 0x00;  //     ->
  ki.swap[10] = 0x00;  // +4  実測値
  ki.swap[11] = 0x00;  //     ->
  ki.swap[12] = 0x00;  // +6  出力値
  ki.swap[13] = 0x00;  //     ->
  ki.swap[14] = 0x00;  // +8  補完動作時の補完速度
  ki.swap[15] = 0x00;  // +9  補完動作終了時のポジション
  ki.swap[16] = 0x00;  //     ->
  ki.swap[17] = 0x00;  // +11 補完動作中の補完弾数
  ki.swap[18] = 0x00;  // +12 補完動作時の動作幅
  ki.swap[19] = 0x00;  //     ->
  ki.swap[20] = 0x00;  // +14 ミキシング1の元となるデータのアドレス
  ki.swap[21] = 0x00;  //     ->
  ki.swap[22] = 0x00;  // +16 ミキシング1の計算方法
  ki.swap[23] = 0x00;  // +17 ミキシング2の元となるデータのアドレス
  ki.swap[24] = 0x00;  //     ->
  ki.swap[25] = 0x00;  // +19 ミキシング2の動作方法
  ki.swap[26] = kondo_checksum(&ki, 26);
  ROS_INFO("registering servo_id %d ...",servo_id);
  return kondo_trx(&ki, 27, 4);
}

int set_serial_servo_trim(unsigned short ram_addr) {
  ram_addr+=2;
  ki.swap[0] = 9;
  ki.swap[1] = RCB4_CMD_MOV;
  ki.swap[2] = RCB4_COM_TO_RAM;
  ki.swap[3] = (unsigned char)(ram_addr>>0);
  ki.swap[4] = (unsigned char)(ram_addr>>8);
  ki.swap[5] = 0x00;
  // 内容
  ki.swap[6]  = 0x00;  // 基準値 (trim)
  ki.swap[7]  = 0x00;
  ki.swap[8] = kondo_checksum(&ki, 8);
  ROS_INFO("setting servo_id's trim to zero...");
  return kondo_trx(&ki, 9, 4);
}

// windowsから一度ROMに書き込んでおくと設定がコピーできる。
// idが同じでポートが違うサーボを一つだけ動かすこともこれで可能になる。
int copy_serial_servo_register_from_rom(unsigned short ram_addr, unsigned char servo_id) {
  unsigned long rom_addr = 0x00626 + (servo_id * 20);
  // ROS_INFO("rom_addr = %x", rom_addr);
  ki.swap[0]  = 11;                               // data size
  ki.swap[1]  = 0x00;                             // MOV
  ki.swap[2]  = 0x03;                             // ROM -> RAM
  ki.swap[3]  = (unsigned char)(ram_addr >> 0);   // RAM addr: lower byte
  ki.swap[4]  = (unsigned char)(ram_addr >> 8);   //           upper byte
  ki.swap[5]  = 0x00;                             // (fixed)
  ki.swap[6]  = (unsigned char)(rom_addr >>  0);  // ROM addr  0- 7 bit
  ki.swap[7]  = (unsigned char)(rom_addr >>  8);  //           8-15 bit
  ki.swap[8]  = (unsigned char)(rom_addr >> 16);  //          16-23 bit
  ki.swap[9]  = 20;                               // reply data size
  ki.swap[10] = kondo_checksum(&ki, 10);          // checksum
  return kondo_trx(&ki, 11, 4);
}


int read_servo_register(char lower, char upper) {
  ki.swap[0] = 10;
  ki.swap[1] = RCB4_CMD_MOV;
  ki.swap[2] = RCB4_RAM_TO_COM;
  ki.swap[3] = 0x00;
  ki.swap[4] = 0x00;
  ki.swap[5] = 0x00;
  ki.swap[6] = lower;
  ki.swap[7] = upper;
  ki.swap[8] = 20;
  ki.swap[9] = kondo_checksum(&ki, 9);
  return kondo_trx(&ki, 10, 23);
}

int register_servo_register_addr(unsigned short register_addr, unsigned char servo_id) {
  unsigned short ics_addr = 0x0044 + 2*servo_id;
  ki.swap[0] = 9;
  ki.swap[1] = RCB4_CMD_MOV;
  ki.swap[2] = RCB4_COM_TO_RAM;
  ki.swap[3] = (unsigned char)(ics_addr >> 0);
  ki.swap[4] = (unsigned char)(ics_addr >> 8);
  ki.swap[5] = 0x00;
  ki.swap[6] = (unsigned char)(register_addr >> 0);
  ki.swap[7] = (unsigned char)(register_addr >> 8);
  ki.swap[8] = kondo_checksum(&ki, 8);
  ROS_INFO("copying ram_addr %hu to address %hu",register_addr, ics_addr);
  return kondo_trx(&ki, 9, 4);
}

int copy_and_register_servo_register(unsigned short ram_addr, unsigned char servo_id) {
  ROS_INFO("registering servo_id %d with ram address %hu ...", servo_id, ram_addr);
  int ret=copy_serial_servo_register_from_rom(ram_addr, servo_id);

  // ROS_INFO("setting servo's serial register at address %hu...",ram_addr);
  // int ret=set_serial_servo_register(ram_addr, servo_id);
  ROS_INFO("setting servo's trim to zero...");
  ret=set_serial_servo_trim(ram_addr);

  ROS_INFO("registering user ram address to ICS designation address...");
  ret=register_servo_register_addr(ram_addr, servo_id);
  return ret;
}

int single_servo_action(unsigned char servo_id, unsigned short position, unsigned char speed) {
  ki.swap[0] = 7;
  ki.swap[1] = 0x0f;
  ki.swap[2] = servo_id;
  ki.swap[3] = speed;
  ki.swap[4] = (position >> 0);
  ki.swap[5] = (position >> 8);
  ki.swap[6] = kondo_checksum(&ki, 6);
  return kondo_trx(&ki, 7, 4);
}

int all_servo_action(unsigned short position[], unsigned char speed) {
  ki.swap[0] = 9+2*KHR_DOF;
  ki.swap[1] = RCB4_CMD_ICS;
  ki.swap[2] = 0b11110000;
  ki.swap[3] = 0b00001111;
  ki.swap[4] = 0b00001000;
  ki.swap[5] = 0b01000000;
  ki.swap[6] = 0b00000000;
  ki.swap[7] = speed;
  for (int i = 0; i < KHR_DOF; i++) {
    ki.swap[8 + (2 * i)] = (position[i] >> 0);
    ki.swap[9 + (2 * i)] = (position[i] >> 8);
  }
  ki.swap[8+2*KHR_DOF] = kondo_checksum(&ki,8+2*KHR_DOF);
  return kondo_trx(&ki, 9+2*KHR_DOF, 4);
}

// Windowsで頭を動かしたときのコマンドを真似してみる。
// これで動く。
int windows_head_move() {
  ki.swap[0] = 0x0A;
  ki.swap[1] = 0x00;
  ki.swap[2] = 0x12;
  ki.swap[3] = 0x06;
  ki.swap[4] = 0x00;
  ki.swap[5] = 0x00;
  ki.swap[6] = 0xF5;
  ki.swap[7] = 0x20;
  ki.swap[8] = 0x00;
  ki.swap[9] = 0x37;
  return kondo_trx(&ki, 10, 4);
}

// 指令値を読み取る。
// 動作途中だと補完動作の指令値が出るっぽい。
unsigned int read_servo_position(unsigned char servo_id) {
  ki.swap[0] = 0x0A;
  ki.swap[1] = 0x00;
  ki.swap[2] = 0x21;  // ICS -> COM
  ki.swap[3] = 0x00;
  ki.swap[4] = 0x00;
  ki.swap[5] = 0x00;
  ki.swap[6] = 0x06;      // offset
  ki.swap[7] = servo_id;  // ics id
  ki.swap[8] = 0x02;      // data size
  ki.swap[9] = kondo_checksum(&ki, 9);
  kondo_trx(&ki, 10, 7);
  return (ki.swap[2]) + (ki.swap[3] << 8);
}

// 実際のサーボ位置を読み取る。
unsigned int read_servo_real_position(unsigned char servo_id) {
  ki.swap[0] = 0x0A;
  ki.swap[1] = 0x00;
  ki.swap[2] = 0x21;  // ICS -> COM
  ki.swap[3] = 0x00;
  ki.swap[4] = 0x00;
  ki.swap[5] = 0x00;
  ki.swap[6] = 0x04;      // offset
  ki.swap[7] = servo_id;  // ics id
  ki.swap[8] = 0x02;      // data size
  ki.swap[9] = kondo_checksum(&ki, 9);
  kondo_trx(&ki, 10, 7);
  return (ki.swap[2]) + (ki.swap[3] << 8);
}

// 登録してないl-shoulder-pはさすがに動かない。
int windows_lshoulderp_move() {
  ki.swap[0] = 0x0A;
  ki.swap[1] = 0x00;
  ki.swap[2] = 0x12;
  ki.swap[3] = 0x06;
  ki.swap[4] = 0x02;
  ki.swap[5] = 0x00;
  ki.swap[6] = 0x54;
  ki.swap[7] = 0x1D;
  ki.swap[8] = 0x00;
  ki.swap[9] = 0xD5;
  return kondo_trx(&ki, 10, 4);
}

// 全サーボのゲイン（KHR用語では「ストレッチ」）を変更
// ゲインは1~127の範囲
// 単一のゲインを渡して全サーボに同じ値をセット。
int change_all_servo_gain(unsigned char gain) {
  gain = gain <   1 ?   1 : gain;
  gain = gain > 127 ? 127 : gain;
  ki.swap[0]  = KHR_DOF+9;
  ki.swap[1]  = 0x12;  // リファレンスは間違っている
  ki.swap[2] = 0b11111111;
  ki.swap[3] = 0b11111111;
  ki.swap[4] = 0b11111111;
  ki.swap[5] = 0b11111111;
  ki.swap[6] = 0b00001111;
  ki.swap[7]  = 0x01;
  for (int i = 0; i < KHR_DOF; i++)
    { ki.swap[8 + i] = gain; }
  ki.swap[KHR_DOF+8] = kondo_checksum(&ki, KHR_DOF+8);
  return kondo_trx(&ki, KHR_DOF+9, 4);
}

// 全サーボのゲイン（KHR用語では「ストレッチ」）を変更
// ゲインは1~127の範囲
// ゲインの配列を渡して個別の値を指定可能。
// 配列の長さは自由。（下から36要素までが送られる。）
int change_all_servo_gain(unsigned char gain_array[], int array_size) {
  for (int i = 0; i < array_size; i++) {
    gain_array[i] = gain_array[i] <   1 ?   1 : gain_array[i];
    gain_array[i] = gain_array[i] > 127 ? 127 : gain_array[i];
  }
  unsigned char msg_size = 9 + array_size;

  ki.swap[0]  = msg_size;
  ki.swap[1]  = 0x12;
  // swap[2] ~ swap[6] にビット列を入れる
  for (int i = 0; i < 5; i++) {
    int tmp_array_size = array_size - (8 * i);
    if (tmp_array_size > 0) {
      // 下から arryay_size 桁が1でそれ以上が0のビット列
      ki.swap[2 + i] = ~(0b11111111 << tmp_array_size);
    } else {
      ki.swap[2 + i] = 0b00000000;
    }
  }
  ki.swap[7]  = 0x01;
  for (int i = 0; i < array_size; i++)
    { ki.swap[8 + i] = gain_array[i]; }
  ki.swap[8 + array_size] = kondo_checksum(&ki, 8 + array_size);
  return kondo_trx(&ki, msg_size, 4);
}

double servo2angle(std::string name, unsigned short position) {
  double angle = (position - 7500) * 90.0 / 2500.0;
  angle = angle * M_PI / 180.0;
  // do some plus/minus correction here
  return angle;
}

unsigned short angle2servo(std::string name, double angle) {
  angle = angle * 180.0 / M_PI;
  // do some plus/minus correction here
  return (unsigned short)(angle * 2500.0 / 90.0) + 7500;
}
