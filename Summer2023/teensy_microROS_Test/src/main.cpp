#include <Arduino.h>

#include <micro_ros_platformio.h>

#include "BerryIMU_v3.h"
#include "Madgwick_Filter.h"
#include "baro_acc_kf.h"
#include "accelGCorrection.h"
#include "EMAFilter.h"
#include "gyro_ekf.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h> //include the message type that needs to be published (teensy data)
#include <micro_ros_utilities/string_utilities.h>

#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/transform_stamped.h>


#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

//constants
#define MICROS_TO_SEC             1000000.0
#define BUFFER_LEN                200

//state machine of teensy agent (the existence of the connection)
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//sensor objects
BerryIMU_v3 BerryIMU;
Madgwick_Filter madgwick;
BaroAccKF kf;
AccelGCorrection accelGCorrection;
GyroEKF gyroEKF;

//sensor and controller rates
#define FAST_SENSOR_LOOP_FREQ           100.0

//timing global variables for each update loop
float lastSensorFastLoopTick = 0.0;

//IMU orentation
float rotation = 0;   //roation matrix angle

//sensor data
float pitch = 0;
float yaw = 0;
float roll = 0;

//pre process for accel before vertical kalman filter
EMAFilter verticalAccelFilter(0.05);

//filter on yaw gyro
EMAFilter yawRateFilter(0.2);

//ROS node
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//ROS publishers
rcl_publisher_t burn_publisher;
rcl_publisher_t imu_publisher;    
// rcl_publisher_t tf_publisher;

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(1000);
  }
}

//String message
std_msgs__msg__String msg;
const char* burn_cream_str = "Got'Em, get the burn cream!"; //const char message
int counter = 0;

//sensor message
sensor_msgs__msg__Imu imu_msg;

//transform message
geometry_msgs__msg__TransformStamped tf_msg;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
  snprintf(msg.data.data, BUFFER_LEN, "%s for the %dth time!", burn_cream_str, counter++);
  msg.data.size = strlen(msg.data.data);
  msg.data.capacity = BUFFER_LEN;
  RCSOFTCHECK(rcl_publish(&burn_publisher, &msg, NULL));
  }
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library wif
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  //this breaks the connection the third time the agent attempts to connect
  
  // //Set the ROS domain ID
  // size_t domain_id = 0;
  // rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  // rcl_init_options_set_domain_id(&init_options, domain_id);
  // RCCHECK(rcl_init_options_init(&init_options, allocator));
  // RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support)); //name the robot

  // create publishers
  RCCHECK(rclc_publisher_init_default(&burn_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/burn_cream"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu"));
  // RCCHECK(rclc_publisher_init_default(&tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped), "/transform"));

  // create timer
  const unsigned int timer_period = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&burn_publisher, &node);
  rcl_publisher_fini(&imu_publisher, &node);
  // rcl_publisher_fini(&tf_publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  state = WAITING_AGENT; //wait for connection
  //initialize
  imu_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  imu_msg.header.frame_id.size = 0;
  imu_msg.header.frame_id.capacity = BUFFER_LEN;

  // tf_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  // tf_msg.header.frame_id.size = 0;
  // tf_msg.header.frame_id.capacity = BUFFER_LEN;

  msg.data.data = (char *) malloc(BUFFER_LEN*sizeof(char));
  msg.data.size = strlen(msg.data.data);
  msg.data.capacity = BUFFER_LEN;

  delay(2000);
}

void loop() {
  //publisher state machine
  // checking if the agent is available 
    switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  //IMU loop
  float dt = micros()/MICROS_TO_SEC-lastSensorFastLoopTick;
  if (dt >= 1.0/FAST_SENSOR_LOOP_FREQ) {
    lastSensorFastLoopTick = micros()/MICROS_TO_SEC;

    //read sensor values and update madgwick
    BerryIMU.IMU_read();
    BerryIMU.IMU_ROTATION(rotation);

    // Serial.print(">x accel:");
    // Serial.println(BerryIMU.AccXraw);
    // Serial.print(">y accel:");
    // Serial.println(BerryIMU.AccYraw);

    madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw,
                             BerryIMU.gyr_rateYraw,
                             BerryIMU.gyr_rateZraw,
                             BerryIMU.AccXraw,
                             BerryIMU.AccYraw,
                             BerryIMU.AccZraw);

    //publish imu data
    snprintf(imu_msg.header.frame_id.data, BUFFER_LEN, "Burn Cream Blimp");
    imu_msg.header.frame_id.size = strlen(msg.data.data);
    imu_msg.header.frame_id.capacity = BUFFER_LEN;

    unsigned int now = micros();
    imu_msg.header.stamp.sec = (unsigned int)((double)now/1000000);
    imu_msg.header.stamp.nanosec = (now % 1000000) * 1000;
        
        //Estimated body orentation (quaternion)
        imu_msg.orientation.w = madgwick.q1;
        imu_msg.orientation.x = madgwick.q2;
        imu_msg.orientation.y = madgwick.q3;
        imu_msg.orientation.z = madgwick.q4;

        // //Estimated body frame angular velocity from gyro
        imu_msg.angular_velocity.x = BerryIMU.gyr_rateXraw;
        imu_msg.angular_velocity.y = BerryIMU.gyr_rateYraw;
        imu_msg.angular_velocity.z = BerryIMU.gyr_rateZraw;

        // //Estimated body frame acceleration from accelerometer
        imu_msg.linear_acceleration.x = BerryIMU.AccXraw;
        imu_msg.linear_acceleration.y = BerryIMU.AccYraw;
        imu_msg.linear_acceleration.z = BerryIMU.AccZraw;
      
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // //publish tranform data
    // snprintf(tf_msg.header.frame_id.data, BUFFER_LEN, "Burn Cream Blimp");
    // tf_msg.header.frame_id.size = strlen(msg.data.data);
    // tf_msg.header.frame_id.capacity = BUFFER_LEN;

    // tf_msg.header.stamp.sec = (unsigned int)((double)now/1000000);
    // tf_msg.header.stamp.nanosec = (now % 1000000) * 1000;

    //     // x,y,z translation
    //     tf_msg.transform.translation.x = 0.0;
    //     tf_msg.transform.translation.y = 0.0;
    //     tf_msg.transform.translation.z = 0.0;

    //     // rotation in quaternion
    //     tf_msg.transform.rotation.w = madgwick.q1;
    //     tf_msg.transform.rotation.x = madgwick.q2;
    //     tf_msg.transform.rotation.y = madgwick.q3;
    //     tf_msg.transform.rotation.z = madgwick.q4;
        
    // RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));


    // get orientation from madgwick
    pitch = madgwick.pitch_final;
    roll = madgwick.roll_final;
    yaw = madgwick.yaw_final;

    //compute the acceleration in the barometers vertical reference frame
    // accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

    //run the prediction step of the vertical velecity kalman filter
    // kf.predict(dt);
    // xekf.predict(dt);
    // yekf.predict(dt);
    // gyroEKF.predict(dt);

    //pre filter accel before updating vertical velocity kalman filter
    // verticalAccelFilter.filter(-accelGCorrection.agz);

    //update vertical velocity kalman filter acceleration
    // kf.updateAccel(verticalAccelFilter.last);

    //update filtered yaw rate
    //  yawRateFilter.filter(BerryIMU.gyr_rateZraw);
    // Serial.print(">yaw rate:");
    // Serial.println(yawRateFilter.last);
    // Serial.print(">Pitch:");
    // Serial.println(pitch);
    // Serial.print(">Roll:");
    // Serial.println(roll);

    //perform gyro update
    // gyroEKF.updateGyro(BerryIMU.gyr_rateXraw*3.14/180, BerryIMU.gyr_rateYraw*3.14/180, BerryIMU.gyr_rateZraw*3.14/180); //deg to rad
    // gyroEKF.updateAccel(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);
    
    // xekf.updateAccelx(-accelGCorrection.agx);
    // xekf.updateGyroX(gyroEKF.pitchRate - gyroEKF.pitchRateB);
    // xekf.updateGyroZ(BerryIMU.gyr_rateZraw);
    
    // yekf.updateAccelx(-accelGCorrection.agy);
    // yekf.updateGyroX(gyroEKF.rollRate - gyroEKF.rollRateB);
    // yekf.updateGyroZ(BerryIMU.gyr_rateZraw);

    
    // Serial.print(">Z:");
    // Serial.println(xekf.z);

    // Serial.print(">Opt:");
    // Serial.println(xekf.opt);

    // Serial.print(">gyro x:");
    // Serial.println(xekf.gyrox);

    // Serial.print(">Ax:");
    // Serial.println(xekf.ax);

    // Serial.print(">Velocity:");
    // Serial.println(xekf.v);

    //print("Yrate", yawRateFilter.last);

    //print("zVel", kf.v);
    // kal_vel.predict_vel();
    // kal_vel.update_vel_acc(-accelGCorrection.agx/9.81, -accelGCorrection.agy/9.81);
  }

}






