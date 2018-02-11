#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
extern "C" {
#include "rcb4.h"
}
#include <unistd.h>
#include "khr_utils.h"
#include <walker_kondo_driver/GetState.h>

extern KondoInstance ki;
unsigned char servo_gain[] = {0x03, 0x03, 0x03, 0x03, 0x03, 0x03};
unsigned char servo_gain_hard[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
sensor_msgs::JointState prev_joint_state, prev_sensor_state;

// 受け取ったJointStateのnameをもとに、
// KHRの低レイヤー表現と同じ要素数・順番に成形したJointStateを返す
// js_outは空っぽの前提
// 同じ名前が複数個あったら最初のもののみ採用
// 無かった関節は0度扱い（要改良）
void sanitiseJointState(const sensor_msgs::JointState::ConstPtr& js_in,
                        sensor_msgs::JointState *js_out)
{
  sensor_msgs::JointState tmp_js;
  const char* joint_name[KHR_DOF] =
    {"r_upper_r_lower_joint","l_upper_l_lower_joint",
    "waist_r_thingy_joint","waist_l_thingy_joint",
    "r_thingy_r_upper_joint","l_thingy_l_upper_joint"};
  for (int i = 0; i < KHR_DOF; i++) {
    const char* tmp_name = joint_name[i];
    double tmp_pos = 0;
    double tmp_eff = 0;
    for (int j = 0; j < js_in->name.size(); j++) {
      if (js_in->name[j] == tmp_name) {
        if (js_in->position.size() == js_in->name.size())
          tmp_pos = js_in->position[j];
        if (js_in->effort.size() == js_in->name.size())
          tmp_eff = js_in->effort[j];
        break;
      }
    }
    js_out->name.push_back(tmp_name);
    if (js_in->position.size() == js_in->name.size())
      js_out->position.push_back(tmp_pos);
    if (js_in->effort.size() == js_in->name.size())
      js_out->effort.push_back(tmp_eff);
  }
}

bool getStateCb(walker_kondo_driver::GetState::Request  &req,
                walker_kondo_driver::GetState::Response &res)
{
  res.joint_state = prev_joint_state;
  res.sensor_state = prev_sensor_state;
  return true;
}

void jsCommandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ros::Time tmp_time = ros::Time::now();
  ROS_INFO("jsCommandCallback start at %d", tmp_time.nsec);
  sensor_msgs::JointState msg_sane;
  sanitiseJointState(msg, &msg_sane);
  if (msg_sane.name.size() != KHR_DOF || msg_sane.position.size() != KHR_DOF) {
    ROS_WARN("[gainCmdCb] invalid joint_state command.");
    return;
  }
  unsigned short position[KHR_DOF];
  for (int i = 0; i < KHR_DOF; i++)
    { position[i] = 7500; }
  for (int servo_num = 0; servo_num < KHR_DOF; servo_num++ ) {
    position[servo_num] = angle2servo(std::string(msg_sane.name[servo_num]), msg_sane.position[servo_num]);
  }
  tmp_time = ros::Time::now();
  ROS_INFO("jsCommandCallback end at %d", tmp_time.nsec);
  int ret=all_servo_action(position, (unsigned char)1);
  return;
}

void gainCommandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState msg_sane;
  sanitiseJointState(msg, &msg_sane);
  if (msg_sane.name.size() != KHR_DOF || msg_sane.effort.size() != KHR_DOF) {
    ROS_WARN("[gainCmdCb] invalid gain command (JointState).");
    return;
  }
  for (int i = 0; i < msg_sane.name.size(); i++) {
    servo_gain[i] = msg_sane.effort[i];
  }
  ROS_INFO("change servo gain.");
  change_all_servo_gain(servo_gain, sizeof(servo_gain) / sizeof(servo_gain[0]));
  return;
}

// argv[1]: 使用するKHRのindex（省略した場合は0）
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kondo_driver");
  ros::NodeHandle n("~");
  ros::Publisher ss_pub = n.advertise<sensor_msgs::JointState>("servo_state", 1000);
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1000);
  ros::Publisher sensor_pub = n.advertise<sensor_msgs::JointState>("sensor_state", 1000);
  ros::Subscriber jscmd_sub = n.subscribe("command/joint_state", 1000, jsCommandCallback);
  ros::Subscriber gaincmd_sub = n.subscribe("command/gain", 1000, gainCommandCallback);
  ros::ServiceServer get_state_srv = n.advertiseService("get_state", getStateCb);
  ros::Rate loop_rate(10);


  // open -------------------------------------------------------------------
  int ret;
  if (argc <= 1) {
    ret = kondo_init(&ki);
  } else {
    ret = kondo_init_index(&ki, atoi(argv[1]));
  }

  if (ret < 0) {
    printf("%s", ki.error);
    exit(-1);
  }
  ki.debug = false;


  // servo on
  ROS_INFO("----- servo on");
  init_servo();

  ROS_INFO("sleep 1 sec...");
  sleep(1);

  ROS_INFO("change servo gain");
  change_all_servo_gain(50);

  int ids[]={6,7,8,9,10,11};

  // move servo
  ROS_INFO("Ready.");

  // DEMO: circular movement
  // unsigned short position=6500;
  // change_all_servo_gain(20);
  // short increment=100;
  // while(ros::ok()){
  //   for(int i=0;i<KHR_DOF;i++){
  //     single_servo_action(ids[i], position, 126);
  //   }
  //   position+=increment;
  //   if (position>8500){
  //     increment=-100;
  //     position=8500;
  //   }
  //   if(position<6500){
  //     increment=100;
  //     position=6500;
  //   }
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  while (ros::ok()) {
    sensor_msgs::JointState ss_msg, js_msg;
    unsigned short position[KHR_DOF];
    const char* name[KHR_DOF] =
      {"r_upper_r_lower_joint","l_upper_l_lower_joint","waist_r_thingy_joint","waist_l_thingy_joint","r_thingy_r_upper_joint","l_thingy_l_upper_joint"};
    for (int servo_num = 0; servo_num < KHR_DOF; servo_num++ ) {
      position[servo_num] = kondo_get_servo_pos((KondoRef)&ki, servo_num);
      ss_msg.position.push_back(position[servo_num]);
      ss_msg.name.push_back(name[servo_num]);
      if (position[servo_num] != 0) {
        js_msg.position.push_back(servo2angle(std::string(name[servo_num]), position[servo_num]));
        js_msg.name.push_back(name[servo_num]);
      } else {
        js_msg.position.push_back(0);
        js_msg.name.push_back(name[servo_num]);
     }
    }
    ss_pub.publish(ss_msg);
    js_pub.publish(js_msg);
    prev_joint_state = js_msg;

    sensor_msgs::JointState sensor_msg;
    const char* sensor_name[4] = {"gyro_p", "gyro_r", "acc_y", "acc_x"};
    for (UINT sensor_idx = 0; sensor_idx < 4; sensor_idx++) {
      int sensor_val;
      kondo_read_analog((KondoRef)&ki, &sensor_val, sensor_idx + 1);
      sensor_msg.name.push_back(sensor_name[sensor_idx]);
      sensor_msg.position.push_back(sensor_val);
    }
    sensor_pub.publish(sensor_msg);
    prev_sensor_state = sensor_msg;

    ros::spin();
    loop_rate.sleep();
  }


  // close ------------------------------------------------------------------
  ret = kondo_close(&ki);
  if (ret < 0)
    error(&ki);

  return 0;
}
