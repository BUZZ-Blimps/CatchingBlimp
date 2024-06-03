//Main include for catching blimp program
#include "catching_blimp.h"

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

//client functions
//TODO: fix these to make them work
// void client_callback(const void * msgin) {
// // Cast response message to expected type
//     test_msgs__srv__BasicTypes_Response *id_msg = (test_msgs__srv__BasicTypes_Response *) msgin;
//     check = id_msg->bool_value;
// }

//send request
// void send_request() {
//     // request
//     int64_t seq;
//     test_msgs__srv__BasicTypes_Request__init(&req);
//     snprintf(req.string_value.data, BUFFER_LEN, "%s", blimpNameSpace.c_str());
//     req.string_value.size = strlen(req.string_value.data);
//     req.string_value.capacity = BUFFER_LEN;
//     delay(2000);  //Sleep a while to ensure DDS matching before sending request
//     RCSOFTCHECK(rcl_send_request(&client, &req, &seq));
// }

//receive response
// void receive_response() {
//     //response
//     delay(2000);
//     RCSOFTCHECK(rcl_take_response(&client,&req_id, &res));
// }

//timer call back
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // snprintf(msg.data.data, BUFFER_LEN, "%s for the %dth time!", blimp_name_str, counter++);
        RCSOFTCHECK(rcl_publish(&identity_publisher, &identity_msg, NULL));
    }
}

void publish_log(const char *message) {
    snprintf(log_msg.data.data, BUFFER_LEN, "%s", message);
    log_msg.data.size = strlen(log_msg.data.data);
    log_msg.data.capacity = BUFFER_LEN;
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

//subscription massage callbacks
//message topics : /auto /baseBarometer/grabbing /killed /motorCommands /shooting /
void auto_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *auto_msg = (const std_msgs__msg__Bool *)msgin;
    if (auto_msg->data) {
        if (blimp_state == manual) {
            publish_log("Activating Auto Mode");
        }
        blimp_state = autonomous;
    } else {
        if (blimp_state == autonomous) {
            publish_log("Going Manual for a Bit...");
        }
        blimp_state = manual;
    }
}

void calibrateBarometer_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *calibration_msg = (const std_msgs__msg__Bool *)msgin;
    calibrateBaro = calibration_msg->data;
    const char * boolAsConstCharPtr = calibrateBaro ? "true" : "false";
    // publish_log(boolAsConstCharPtr);

    // Barometer Calibration
    if (calibrateBaro == true) {
        baroCalibrationOffset = BerryIMU.comp_press - baseBaro;

        std::string floatAsString = std::to_string(BerryIMU.comp_press);
        const char* floatAsConstCharPtr = floatAsString.c_str();
        publish_log(floatAsConstCharPtr);
        std::string floatAsString2 = std::to_string(baseBaro);
        const char* floatAsConstCharPtr2 = floatAsString2.c_str();
        publish_log(floatAsConstCharPtr2);
        std::string floatAsString3 = std::to_string(baroCalibrationOffset);
        const char* floatAsConstCharPtr3 = floatAsString3.c_str();
        publish_log(floatAsConstCharPtr3);
        publish_log("Calibrating Barometer");
    }
}

void baro_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64 *baro_msg = (const std_msgs__msg__Float64 *)msgin;
    baseBaro = baro_msg->data;
    
    //heartbeat
    //update last message time
    lastMsgTime = micros()/MICROS_TO_SEC;

    //If teensy comes out of lost blimp_state, put it in manual control mode
    if (blimp_state == lost) {
        blimp_state = manual;
    }
}

void grab_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *grab_msg = (const std_msgs__msg__Bool *)msgin;
    if (grabCom == 0 && grab_msg->data) {
        grabCom = 1;
        publish_log("Going for a catch...");
    } else if (grabCom == 1 && !grab_msg->data) {
        grabCom = 0;
        publish_log("Hopefully I got a balloon!");
    }
}

void kill_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *kill_msg = (const std_msgs__msg__Bool *)msgin;

    if (kill_msg->data == true) {
        publish_log("I'm ded xD");
        motorControl.update(0,0,0,0,0);
        bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
        leftGimbal.updateGimbal(leftReady && rightReady);
        rightGimbal.updateGimbal(leftReady && rightReady);
    }
}

void shoot_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Bool *shoot_msg = (const std_msgs__msg__Bool *)msgin;
    if (shootCom == 0 && shoot_msg->data) {
        shootCom = 1;
        publish_log("I'm shooting my shot...");
    } else if (shootCom == 1 && !shoot_msg->data) {
        shootCom = 0;
        publish_log("What a shot!");
    }
}

void motor_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *motor_msg = (const std_msgs__msg__Float64MultiArray *)msgin;

    //commands from basestation
    forward_msg = motor_msg->data.data[3];
    up_msg = motor_msg->data.data[1];
    yaw_msg = motor_msg->data.data[0];
    translation_msg = motor_msg->data.data[2];

    // char motorCommands[100];  // Size depending on the expected maximum length of your combined string
    // sprintf(motorCommands, "Teensy Motor Commands\nYaw: %.2f\nUp: %.2f\nTranslation: %.2f\nForward: %.2f\n", yaw_msg, up_msg, translation_msg, forward_msg);
    // publish_log(motorCommands);
}

void goal_color_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int64 *goal_color_msg = (const std_msgs__msg__Int64 *)msgin;
    int goal_color = goal_color_msg->data;
    if (goalColor != orange && goal_color == 0) {
        goalColor = orange;
        publish_log("Goal Color changed to Orange");
    } else if (goalColor != yellow && goal_color == 1) {
        goalColor = yellow;
        publish_log("Goal Color changed to Yellow");
    }
}

void avoidance_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *avoidance_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        avoidance[i] = avoidance_msg->data.data[i];
    }
}

void targets_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *targets_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    // object of interest with xyz (3 elements in total)
    for (size_t i = 0; i < 3; ++i) {
        targets[i] = targets_msg->data.data[i];
    }
}

void pixels_subscription_callback(const void *msgin)
{
    const std_msgs__msg__Int64MultiArray *pixels_msg = (const std_msgs__msg__Int64MultiArray *)msgin;
    //3 objects with xyz (9 elements in total)
    for (size_t i = 0; i < 9; ++i) {
        pixels[i] = pixels_msg->data.data[i];
    }
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, blimpNameSpace.c_str(), "", &support)); //name the robot

    //create client
    // RCCHECK(rclc_client_init_default(&client, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(test_msgs, srv, BasicTypes), "/BlimpID"));

    // create publishers (7 right now)
    RCCHECK(rclc_publisher_init_default(&identity_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/identify"));
    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), (blimpNameSpace + "/imu").c_str()));
    RCCHECK(rclc_publisher_init_default(&debug_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray ), (blimpNameSpace + "/debug").c_str()));
    RCCHECK(rclc_publisher_init_default(&height_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), (blimpNameSpace + "/height").c_str()));
    RCCHECK(rclc_publisher_init_default(&z_velocity_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), (blimpNameSpace + "/z_velocity").c_str()));
    RCCHECK(rclc_publisher_init_default(&state_machine_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), (blimpNameSpace + "/state_machine").c_str()));
    RCCHECK(rclc_publisher_init_default(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), (blimpNameSpace + "/log").c_str()));

    //create subscribers (10 right now)
    //Base station
    RCCHECK(rclc_subscription_init_default(&auto_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/auto").c_str()));
    RCCHECK(rclc_subscription_init_default(&baseBarometer_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), (blimpNameSpace + "/baseBarometer").c_str()));
    RCCHECK(rclc_subscription_init_default(&calibrateBarometer_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/calibrateBarometer").c_str()));
    RCCHECK(rclc_subscription_init_default(&grabber_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/grabbing").c_str()));
    RCCHECK(rclc_subscription_init_default(&shooter_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/shooting").c_str()));
    RCCHECK(rclc_subscription_init_default(&motor_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), (blimpNameSpace + "/motorCommands").c_str()));
    RCCHECK(rclc_subscription_init_default(&kill_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), (blimpNameSpace + "/killed").c_str()));
    RCCHECK(rclc_subscription_init_default(&goal_color_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), (blimpNameSpace + "/goal_color").c_str()));

    //Offboard ML
    RCCHECK(rclc_subscription_init_default(&targets_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), (blimpNameSpace + "/targets").c_str()));
    RCCHECK(rclc_subscription_init_default(&pixels_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray), (blimpNameSpace + "/pixels").c_str()));
    RCCHECK(rclc_subscription_init_default(&avoidance_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray), (blimpNameSpace + "/avoidance").c_str()));

    // create heartbeat (identity) timer (period 1 second)
    const unsigned int timer_period = 1000;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_period), timer_callback));

    // create executor
    executor_pub = rclc_executor_get_zero_initialized_executor();
    executor_sub = rclc_executor_get_zero_initialized_executor();
    // executor_srv = rclc_executor_get_zero_initialized_executor();

    //init executors
    // RCCHECK(rclc_executor_init(&executor_srv, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 11, &allocator));

    //add client  
    // RCCHECK(rclc_executor_add_client(&executor_srv, &client, &res, client_callback));
    // send_request();
    // receive_response();

    //add all subscriptions (9)
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &auto_subscription, &auto_msg, &auto_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &baseBarometer_subscription, &baro_msg, &baro_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &calibrateBarometer_subscription, &calibration_msg, &calibrateBarometer_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &grabber_subscription, &grab_msg, &grab_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &shooter_subscription, &shoot_msg, &shoot_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &kill_subscription, &kill_msg, &kill_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &motor_subscription, &motor_msg, &motor_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &goal_color_subscription, &goal_color_msg, &goal_color_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &targets_subscription, &targets_msg, &targets_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &pixels_subscription, &pixels_msg, &pixels_subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &avoidance_subscription, &avoidance_msg, &avoidance_subscription_callback, ON_NEW_DATA));

    return true;
}

void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    //finish publishers
    rcl_publisher_fini(&identity_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&debug_publisher, &node);
    rcl_publisher_fini(&height_publisher, &node);
    rcl_publisher_fini(&z_velocity_publisher, &node);
    rcl_publisher_fini(&state_machine_publisher, &node);
    rcl_publisher_fini(&log_publisher, &node);

    //finish subscriptions
    rcl_subscription_fini(&auto_subscription, &node);
    rcl_subscription_fini(&baseBarometer_subscription, &node);
    rcl_subscription_fini(&calibrateBarometer_subscription, &node);
    rcl_subscription_fini(&grabber_subscription, &node);
    rcl_subscription_fini(&shooter_subscription, &node);
    rcl_subscription_fini(&motor_subscription, &node); 
    rcl_subscription_fini(&kill_subscription, &node);
    rcl_subscription_fini(&goal_color_subscription, &node);
    rcl_subscription_fini(&targets_subscription, &node);
    rcl_subscription_fini(&pixels_subscription, &node);
    rcl_subscription_fini(&avoidance_subscription, &node);

    //finish client
    // rcl_client_fini(&client, &node);

    rcl_timer_fini(&timer);
    // rclc_executor_fini(&executor_srv);
    rclc_executor_fini(&executor_pub);
    rclc_executor_fini(&executor_sub);

    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library wifi
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3

//pulse function for adding ultrasonic
void Pulse() {
    if (digitalRead(interruptPin) == HIGH) {
        // start measuring
        pulseInTimeBegin = micros();
    }
    else {
        // stop measuring
        pulseInTimeEnd = micros();
        newPulseDurationAvailable = true;
    }
}

float searchDirection() {
    int rand = random(0, 10);
    float binary = 1.0;
    if (rand <= 4){
        binary = 1.0;
    }else if (rand > 4){
        binary = -1.0;
    }
    return binary;
}

//set up

void setup() {
    //old code
    //start serial connection
    //Serial.begin(115200);
    //set_microros_serial_transports(Serial); //to pi

    //agent_state_ = WAITING_AGENT; //wait for connection
    //Serial1.begin(115200);
    //old code

    //Start Servos
    Servo_L.servo_setup(0);
    Servo_R.servo_setup(2);
    Servo_L.servo_angle(0);
    Servo_R.servo_angle(180);

    //Start Brushless
    Brushless_L.brushless_setup(5);
    Brushless_R.brushless_setup(16);
    Brushless_L.brushless_thrust(1500);
    Brushless_R.brushless_thrust(1500);

    //Start IMU
    BerryIMU.OPI_IMU_Setup();

    //initialize messages
    //service message
    // req.string_value.data = (char *)malloc(BUFFER_LEN * sizeof(char));

    //publisher/subscriber messages
    //See https://micro.ros.org/docs/tutorials/advanced/handling_type_memory
    //String type
    identity_msg.data.capacity = BUFFER_LEN;
    identity_msg.data.data = (char *) malloc(identity_msg.data.capacity*sizeof(char));
    identity_msg.data.size = 0;

    //Once identity_msg is initialized, we can set it to its constant value (blimp ID / namespace)
    snprintf(identity_msg.data.data, BUFFER_LEN, "%s", blimpNameSpace.c_str());
    identity_msg.data.size = strlen(identity_msg.data.data);
    identity_msg.data.capacity = BUFFER_LEN;

    log_msg.data.capacity = BUFFER_LEN;
    log_msg.data.data = (char *) malloc(log_msg.data.capacity*sizeof(char));
    log_msg.data.size = 0;

    //IMU Header
    imu_msg.header.frame_id.capacity = BUFFER_LEN;
    imu_msg.header.frame_id.data = (char *) malloc(BUFFER_LEN*sizeof(char));
    imu_msg.header.frame_id.size = 0;

    //Float64MultiArray type
    motor_msg.data.capacity = 4;
    motor_msg.data.data = (double *) malloc(motor_msg.data.capacity*sizeof(double));
    motor_msg.data.size = 0;

    //Initialize MultiArray layout and inner members
    motor_msg.layout.dim.capacity = 100;
    motor_msg.layout.dim.size = 0;
    motor_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(motor_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < motor_msg.layout.dim.capacity; i++){
        motor_msg.layout.dim.data[i].label.capacity = 20;
        motor_msg.layout.dim.data[i].label.size = 0;
        motor_msg.layout.dim.data[i].label.data = (char*) malloc(motor_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    debug_msg.data.capacity = BUFFER_LEN;
    debug_msg.data.data = (double *) malloc(debug_msg.data.capacity*sizeof(double));
    debug_msg.data.size = sizeof(debug_msg.data.data);
    
    debug_msg.layout.dim.capacity = 100;
    debug_msg.layout.dim.size = 0;
    debug_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(debug_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < debug_msg.layout.dim.capacity; i++){
        debug_msg.layout.dim.data[i].label.capacity = 20;
        debug_msg.layout.dim.data[i].label.size = 0;
        debug_msg.layout.dim.data[i].label.data = (char*) malloc(debug_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    targets_msg.data.capacity = MAX_TARGETS;
    targets_msg.data.data = (double *) malloc(targets_msg.data.capacity*sizeof(double));
    targets_msg.data.size = sizeof(targets_msg.data.data);

    targets_msg.layout.dim.capacity = 100;
    targets_msg.layout.dim.size = 0;
    targets_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(targets_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < targets_msg.layout.dim.capacity; i++){
        targets_msg.layout.dim.data[i].label.capacity = 20;
        targets_msg.layout.dim.data[i].label.size = 0;
        targets_msg.layout.dim.data[i].label.data = (char*) malloc(targets_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    //Todo: Set capacity on these guys to something reasonable
    avoidance_msg.data.capacity = 100;
    avoidance_msg.data.data = (double *) malloc(avoidance_msg.data.capacity*sizeof(double));
    avoidance_msg.data.size = sizeof(avoidance_msg.data.data);

    avoidance_msg.layout.dim.capacity = 100;
    avoidance_msg.layout.dim.size = 0;
    avoidance_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(avoidance_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < avoidance_msg.layout.dim.capacity; i++){
        avoidance_msg.layout.dim.data[i].label.capacity = 20;
        avoidance_msg.layout.dim.data[i].label.size = 0;
        avoidance_msg.layout.dim.data[i].label.data = (char*) malloc(avoidance_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    //Int64MultiArray
    pixels_msg.data.capacity = 100;
    pixels_msg.data.data = (int64_t *) malloc(pixels_msg.data.capacity*sizeof(int64_t));
    pixels_msg.data.size = sizeof(pixels_msg.data.data);
    
    pixels_msg.layout.dim.capacity = 100;
    pixels_msg.layout.dim.size = 0;
    pixels_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(pixels_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for(size_t i = 0; i < pixels_msg.layout.dim.capacity; i++){
        pixels_msg.layout.dim.data[i].label.capacity = 20;
        pixels_msg.layout.dim.data[i].label.size = 0;
        pixels_msg.layout.dim.data[i].label.data = (char*) malloc(pixels_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    //WDW I took this out, seems that there is no need for it
    // delay(2000);

    //time out
    firstMessageTime = micros()/MICROS_TO_SEC;
    
    //initialize
    ballGrabber.ballgrabber_init(GATE_S, PWM_G);
    leftGimbal.gimbal_init(L_Yaw, L_Pitch, PWM_L, 25, 30, MIN_MOTOR, MAX_MOTOR, 45, 0.5);
    rightGimbal.gimbal_init(R_Yaw, R_Pitch, PWM_R, 25, 30, MIN_MOTOR, MAX_MOTOR, 135, 0.5);
}

void update_agent_state() {
    //-------------------------MICRO ROS PUBLISHER and SUBSCRIBERS--------------------------------------
    // publisher blimp_state machine
    // checking if the agent is available 
    switch (agent_state_) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(100, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            agent_state_ = (create_entities() == true) ? AGENT_CONNECTED : WAITING_AGENT;
            if (agent_state_ == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED: //the microros agent is connected to teensy
            EXECUTE_EVERY_N_MS(1000, agent_state_ = (RMW_RET_OK == rmw_uros_ping_agent(10, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (agent_state_ == AGENT_CONNECTED) {
                // rclc_executor_spin_some(&executor_srv, RCL_MS_TO_NS(10));
                rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10));
                rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            agent_state_ = WAITING_AGENT;
            break;

        default:
            break;
    }
}

//Main loop
void loop() {
    double loop_time = micros()/MICROS_TO_SEC;

    //Update the microros agent blimp_state machine
    update_agent_state();

    //Check to make sure the microROS agent is connected before doing anything else
    if (agent_state_ == AGENT_CONNECTED) {
        if (!last_connected) {
            //Reconnected
            last_connected = true;

            //Delay for 1 second before publishing anything
            delay(1000);

            publish_log("Connected to MicroROS Agent");
        }
    } else {
        if (last_connected) {
            //Lost connection
            last_connected = false;
        }

        //Not connected to microROS agent, so don't do anything else
        return;
    }

    //----------------------------------LOST---------------------------------------
    //Switch to lost blimp_state if we lose connection to the base station
    if (loop_time - lastMsgTime > TEENSY_WAIT_TIME) {
        publish_log("I am lost:(");
        blimp_state = lost;
    }

    //Log messages for lost blimp_state
    if (blimp_state == lost && !last_lost) {
        last_lost = true;
        publish_log("Lost Connection to Basestation");
    } else if (blimp_state != lost && last_lost) {
        last_lost = false;
        publish_log("Connected to Basestation");
    }

    //----------------------------IMU LOOP-----------------------------------------
    //compute accel, gyro, madwick loop time at the set freqency
    
    double dt = loop_time-lastSensorFastLoopTick;
    if (dt >= 1.0/FAST_SENSOR_LOOP_FREQ) {

        lastSensorFastLoopTick = loop_time;

        //read sensor values and update madgwick
        BerryIMU.IMU_read();
        BerryIMU.IMU_ROTATION(rotation); // Rotate IMU
        madgwick.Madgwick_Update(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, BerryIMU.gyr_rateZraw, BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw);

        //publish imu data
        snprintf(imu_msg.header.frame_id.data, BUFFER_LEN, blimpNameSpace.c_str());
        imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
        imu_msg.header.frame_id.capacity = BUFFER_LEN;

        unsigned int now = micros();
        imu_msg.header.stamp.sec = (unsigned int)((double)now/1000000);
        imu_msg.header.stamp.nanosec = (now % 1000000) * 1000;

        //Estimated body orentation (quaternion)
        imu_msg.orientation.w = madgwick.q1;
        imu_msg.orientation.x = madgwick.q2;
        imu_msg.orientation.y = madgwick.q3;
        imu_msg.orientation.z = madgwick.q4;

        //Estimated body frame angular velocity from gyro
        imu_msg.angular_velocity.x = BerryIMU.gyr_rateXraw;
        imu_msg.angular_velocity.y = BerryIMU.gyr_rateYraw;
        imu_msg.angular_velocity.z = BerryIMU.gyr_rateZraw;

        //Estimated body frame acceleration from accelerometer
        imu_msg.linear_acceleration.x = BerryIMU.AccXraw;
        imu_msg.linear_acceleration.y = BerryIMU.AccYraw;
        imu_msg.linear_acceleration.z = BerryIMU.AccZraw;

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

        //get orientation from madgwick
        pitch = madgwick.pitch_final;
        roll = madgwick.roll_final;
        yaw = madgwick.yaw_final;

        //compute the acceleration in the barometers vertical reference frame
        accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

        //run the prediction step of the vertical velecity kalman filter
        kf.predict(dt);
        // xekf.predict(dt);
        // yekf.predict(dt);
        gyroEKF.predict(dt);

        //pre filter accel before updating vertical velocity kalman filter
        verticalAccelFilter.filter(accelGCorrection.agz);

        //update vertical velocity kalman filter acceleration
        kf.updateAccel(verticalAccelFilter.last);

        //update filtered yaw rate
        yawRateFilter.filter(BerryIMU.gyr_rateZraw);

        //perform gyro update
        // gyroEKF.updateGyro(BerryIMU.gyr_rateXraw*3.14/180, BerryIMU.gyr_rateYraw*3.14/180, BerryIMU.gyr_rateZraw*3.14/180);
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

        // Serial.print(">Yrate:");
        // Serial.println(yawRateFilter.last);

        // Serial.print(">zVel:");
        // Serial.println(kf.v);

        // kal_vel.predict_vel();
        // kal_vel.update_vel_acc(-accelGCorrection.agx/9.81, -accelGCorrection.agy/9.81);
    }

    //-----------------------------BARO LOOP-----------------------------------------------
    //update barometere at set barometere frequency
    dt = loop_time-lastBaroLoopTick;
    if (dt >= 1.0/BARO_LOOP_FREQ) {
        lastBaroLoopTick = loop_time;

        //get most current imu values
        BerryIMU.IMU_read();

        //update kalman with uncorreced barometer data
        kf.updateBaro(BerryIMU.alt);

        //compute the corrected height with base station baro data and offset
        if (baseBaro != 0) {
            actualBaro = 44330 * (1 - pow(((BerryIMU.comp_press - baroCalibrationOffset)/baseBaro), (1/5.255))); // In meters Base Baro is the pressure

            //publish Height
            height_msg.data = actualBaro;
            // height_msg.data = BerryIMU.comp_press;  //only for debug
            RCSOFTCHECK(rcl_publish(&height_publisher, &height_msg, NULL));

        } else {
            actualBaro = 1000;
        }

        // Add Z Velocity to a Message and Publish
        z_velocity_msg.data = kf.v;

        // if (check == false){
        //   z_velocity_msg.data = 0;
        // }else{
        //   z_velocity_msg.data = 0;
        // }

        RCSOFTCHECK(rcl_publish(&z_velocity_publisher, &z_velocity_msg, NULL));

        // xekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
        // yekf.updateBaro(CEIL_HEIGHT_FROM_START-actualBaro);
    }

    //---------------------OPTICAL FLOW LOOP-----------------------------------------
    // //Optical flow update
    //read buffer for optical flow
    // Flow.read_buffer();
    // dt = micros()/MICROS_TO_SEC-lastOpticalLoopTick;
    // if (dt > 1.0/OPTICAL_LOOP_FREQ) {
    //   lastOpticalLoopTick = micros()/MICROS_TO_SEC;

    //   Flow.update_flow(BerryIMU.gyr_rateXraw, BerryIMU.gyr_rateYraw, 1);

    //   //change to blimp coordinates
    //   float x_opt = (float)Flow.y_motion/dt;
    //   float y_opt = (float)Flow.x_motion/dt;

    //   xekf.updateOptical(x_opt);
    //   yekf.updateOptical(y_opt);

    //   // Serial.print(">X Velocity:");
    //   // Serial.println(Flow.x_motion_comp);

    //   // accelGCorrection.updateData(BerryIMU.AccXraw, BerryIMU.AccYraw, BerryIMU.AccZraw, pitch, roll);

    //   // kal_vel.update_vel_optical(Flow.x_motion_comp, Flow.y_motion_comp);

    //   // Serial.print(">X Velocity est:");
    //   // Serial.println(kal_vel.x_vel_est);

    // }

    //compute slower sensors (if any)
    //use same if statement structure

    //----------------------------STATE MACHINE LOOP-------------------------------
    //compute blimp_state machine and motor control at specified frequency
    dt = loop_time-lastStateLoopTick;
    if (dt >= 1.0/STATE_MACHINE_FREQ) {

        lastStateLoopTick = loop_time;

        //control inputs
        float forwardCom = 0.0;
        float upCom = 0.0;
        float yawCom = 0.0;
        float translationCom = 0.0;

        //object avoidence commands to overide search and computer vition
        float forwardA = 0.0;
        float upA = 0.0;
        float yawA = 0.0;

        //set avoidence command based on quadrant that contains object to avoid
        switch (quadrant) {
        case 1:
            forwardA = -FORWARD_AVOID;
            upA = -UP_AVOID;
            yawA = -YAW_AVOID;
            break;
        case 2:
            forwardA = -FORWARD_AVOID;
            upA = -UP_AVOID;
            yawA = 0;
            break;
        case 3:
            forwardA = -FORWARD_AVOID;
            upA = -UP_AVOID;
            yawA = YAW_AVOID;
            break;
        case 4:
            forwardA = -FORWARD_AVOID;
            upA = 0;
            yawA = -YAW_AVOID;
            break;
        case 5:
            forwardA = -FORWARD_AVOID;
            upA = 0;
            yawA = 0;
            break;
        case 6:
            forwardA = -FORWARD_AVOID;
            upA = 0;
            yawA = YAW_AVOID;
            break;
        case 7:
            forwardA = -FORWARD_AVOID;
            upA = UP_AVOID;
            yawA = -YAW_AVOID;
            break;
        case 8:
            forwardA = -FORWARD_AVOID;
            upA = UP_AVOID;
            yawA = 0;
            break;
        case 9:
            forwardA = -FORWARD_AVOID;
            upA = UP_AVOID;
            yawA = YAW_AVOID;
            break;
        default:
            break;
        }

        //from base station
        //compute blimp_state machine
        if (blimp_state == manual) {
            //get manual data
            //all motor commands are between -1 and 1
            //set max yaw command to 120 deg/s

            yawCom = -yaw_msg*120;

            if (USE_EST_VELOCITY_IN_MANUAL == true){
                //set max velocities 2 m/s
                upCom = up_msg*2.0;
                forwardCom = forward_msg*2.0;
                translationCom = translation_msg*2.0;
            } else {
                //normal mapping using max esc command 
                // upCom = up_msg*2.0; //PID used and maxed out at 2m/s
                upCom = -up_msg*500.0; //up is negative
                // upCom = -up_msg*500.0-0.5*pitch; //pitch correction? (pitch in degrees, conversion factor command/degree)
                forwardCom = forward_msg*500.0;
                translationCom = translation_msg*500.0;
            }

            //check if shooting should be engaged
            //this block switches the blimp_state to the oposite that it is currently in
            if (shoot != shootCom) {
                shoot = shootCom;

                //change shoot blimp_state
                if (ballGrabber.state == 2) {
                    //stop shooting
                    ballGrabber.closeGrabber(blimp_state);
                } else {
                    //reset catch counter
                    catches = 0;

                    //go back to searching
                    auto_state = searching;
                    // searching timer
                    searchingTimeStart = millis();
                    searchYawDirection = searchDirection();  //randomize the search direction

                    //start shooting
                    ballGrabber.shoot(blimp_state);
                }
            //check if grabbing should be engaged
            //this block switches the blimp_state to the oposite that it is currently in
            } else if (grab != grabCom) {
                grab = grabCom;

                //change grab blimp_state
                if (ballGrabber.state == 0) {
                    ballGrabber.openGrabber(blimp_state);
                } else {
                    ballGrabber.closeGrabber(blimp_state);

                    //increase catch counter
                    catches++;

                    //start catch timmer
                    if (catches >= 1) {
                        lastCatch = micros()/MICROS_TO_SEC;
                    }
                }
            }
        } else if (blimp_state == autonomous) {
            //filter target data
            // float tx = 0;
            // float ty = 0;
            float tx = 0;
            float ty = 0;
            float tz = 0;
            float temp_tx = 0;
            float temp_ty = 0;
            // float area = 0;

            //new target (empty target)
            std::vector<double> detected_target;

            //if a target is seen
            if (targets[2] != 1000){
                float rawZ = targets[2]; // distance
                tx = xFilter.filter(static_cast<float>(targets[0]));
                ty = yFilter.filter(static_cast<float>(targets[1]));
                tz = zFilter.filter(rawZ);
                
                // tx = static_cast<float>(targets[0]);
                // ty = static_cast<float>(targets[1]);
                // tz = rawZ;

                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
                
            } else {
                // no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(4);
                // areaFilter.filter(0);
            }
            
            /*
            //update targets data if any target exists
            //TODO: add area to verify distance 
            //balloon 
            if (targets[2] != 1000 && (auto_state == searching || auto_state == approach || auto_state == catching)) {
                float rawZ = targets[2]; //balloon distance
                // update filtered target coordinates (3D space, with center of camera as (0,0,0))
                // tx = xFilter.filter(targets[0]); (3D)
                // ty = yFilter.filter(targets[1]);
                tx = xFilter.filter(static_cast<float>(pixels[0]));
                ty = yFilter.filter(static_cast<float>(pixels[1]));
                tz = zFilter.filter(rawZ);
                // area = areaFilter.filter(target[0][3]);
                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
            } else {
                //no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(0);
                // areaFilter.filter(0);
            }

            //orange goal
            //in goal scoring stages 
            if (targets[5] != 1000 && goalColor == orange && (auto_state == goalSearch || auto_state == approachGoal || auto_state == scoringStart)) {
                float rawZ = targets[5]; //balloon distance
                //update filtered target coordinates (3D space, with center of camera as (0,0,0))
                // tx = xFilter.filter(targets[3]);
                // ty = yFilter.filter(targets[4]); 
                tx = xFilter.filter(static_cast<float>(pixels[3]));
                ty = yFilter.filter(static_cast<float>(pixels[4])); 
                tz = zFilter.filter(rawZ);
                // area = areaFilter.filter(target[0][3]);
                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
            } else {
                //no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(0);
                // areaFilter.filter(0);
            }

            //yellow goal
            if (targets[8] != 1000 && goalColor == yellow && (auto_state == goalSearch || auto_state == approachGoal || auto_state == scoringStart)) {
                float rawZ = targets[8]; //balloon distance
                //update filtered target coordinates (3D space, with center of camera as (0,0,0))
                // tx = xFilter.filter(targets[6]);
                // ty = yFilter.filter(targets[7]);
                tx = xFilter.filter(static_cast<float>(pixels[6]));
                ty = yFilter.filter(static_cast<float>(pixels[7]));
                tz = zFilter.filter(rawZ);
                // area = areaFilter.filter(target[0][3]);
                detected_target.push_back(tx);
                detected_target.push_back(ty);
                detected_target.push_back(tz);
            } else {
                //no target, set to default value
                xFilter.filter(0);
                yFilter.filter(0);
                zFilter.filter(0);
                // areaFilter.filter(0);
            }
            */


            //test target message

            // if (target.size() != 0){
            // debug_msg.data.data[0] = target[0];
            // debug_msg.data.data[1] = target[1];
            // debug_msg.data.data[2] = target[2];
            // debug_msg.data.size = 3;
            // }

            // RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

            //modes for autonomous behavior
            switch (auto_state) {
                //blimp_state machine
                case searching: {

                    //check if goal scoring should be attempted
                    if (catches >= 1 && ((micros()/MICROS_TO_SEC - lastCatch) >= (MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*GAME_BALL_WAIT_TIME_PENALTY))) {
                        catches = TOTAL_ATTEMPTS;
                        auto_state = goalSearch;
                        goalYawDirection = searchDirection();  //randomize search direction
                        break;
                    }

                    if (catches >= TOTAL_ATTEMPTS) {
                        auto_state = goalSearch;
                        goalYawDirection = searchDirection();  //randomize search direction
                        break;
                    }

                    //begin search pattern spinning around at different heights
                    if (detected_target.size() == 0) {

                        //keep ball grabber closed
                        // ballGrabber.closeGrabber();

                        //use object avoidence
                        double avoidanceMinVal = 1000.0; // Initialize 
                        int avoidanceMinIndex = 10;

                        // Iterate through the vector to find the minimum value and its index
                        // find the minimum distance and its corresponding quadrant number (1-9)
                        for (int i = 0; i < 9; ++i) {
                            if (avoidance[i] < avoidanceMinVal) {
                                avoidanceMinVal = avoidance[i]; //distance
                                avoidanceMinIndex = i+1; //quadrant number
                            }
                        }

                        //set the avoidance quadrant only when avoidance range is triggered
                        if (avoidanceMinVal < AVOID_TRIGGER){
                            //update quadrant
                            quadrant = avoidanceMinIndex;
                        } else {
                            //safe
                            //update quadrant
                            quadrant = 10;
                        }

                        //avoding obstacle
                        if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {

                            //overide search commands
                            yawCom = yawA;
                            forwardCom = forwardA;
                            upCom = upA;

                        } else {
                            //search behavior (no detected_target)
                            //spin in a small circle looking for a game ball
                            //randomize the diretion selection
                            // yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //upCom = 0;    //is overriden later, defined here as a safety net
                            // forwardCom = GAME_BALL_FORWARD_SEARCH;
                            
                            // Timeline:
                            // SearchingStart -> +18 seconds -> +20 seconds -> restart searching
                            double elapsedSearchingTime = millis() - searchingTimeStart;
                            std::string message = "elapsedSearchTime=" + std::to_string(elapsedSearchingTime) + "s.";
                            if(elapsedSearchingTime < searchTime){
                                backingUp = false;
                                yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                                forwardCom = GAME_BALL_FORWARD_SEARCH;

                                if (actualBaro > CEIL_HEIGHT) {
                                    upCom = GAME_BALL_VERTICAL_SEARCH; //down
                                }else if (actualBaro < FLOOR_HEIGHT) {
                                    upCom = -GAME_BALL_VERTICAL_SEARCH;  //up
                                }else{
                                    upCom = 0;
                                }
                            }else if(elapsedSearchingTime < searchTime + backupTime){
                                if(!backingUp){
                                    backingUp = true;
                                    searchYawDirection = searchDirection();
                                }   
                                message += " Backup!";
                                
                                yawCom = 35*searchYawDirection;
                                forwardCom = -240;
                                upCom = -100;
                            }else{
                                message += " Reset!";
                                searchingTimeStart = millis();
                            }
                            publish_log(message.c_str());

                            // Prolly hit a net
                            // if (20000 < millis()- searchingTimeStart){
                            //     double backupTimer = millis();

                            //     std::string message = "20+ seconds.";

                            //     if (2200 > millis() - backupTimer) {
                            //         message += " first 2 seconds.";
                            //         searchYawDirection = searchDirection();
                            //         yawCom = 60*searchYawDirection;
                            //         upCom = -50;    //is overriden later, defined here as a safety net
                            //         forwardCom = -200;
                            //     }

                            //     searchingTimeStart = millis(); 
                            //     publish_log(message.c_str());
                            // }

                            //move up and down within the set boundry
                            // if (actualBaro > CEIL_HEIGHT) {
                            //     // if (wasUp) wasUp = false;
                            //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //     upCom = GAME_BALL_VERTICAL_SEARCH; //down
                            //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                            // }

                            // if (actualBaro < FLOOR_HEIGHT) {
                            //     // if (!wasUp) wasUp = true;
                            //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //     upCom = -GAME_BALL_VERTICAL_SEARCH;  //up
                            //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                            // }

                            // if (actualBaro <= CEIL_HEIGHT && actualBaro >=FLOOR_HEIGHT) {
                            //     // if (wasUp) wasUp = false;
                            //     yawCom = GAME_BALL_YAW_SEARCH*searchYawDirection;
                            //     upCom = 0;
                            //     forwardCom = GAME_BALL_FORWARD_SEARCH;
                            // }
                        }

                    } else {
                        //move to approaching game ball
                        auto_state = approach;
                        //start approaching timer
                        approachTimeStart = millis();
                    }
                    break;
                } case approach: {
                    // max time to approach
                    if (approachTimeMax < millis() - approachTimeStart) {
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                    }

                   //check if target is still valid
                    if (detected_target.size() > 0) {
                        //seeing a target
                        //add memory
                        temp_tx = tx;
                        temp_ty = ty;
                        catchMemoryTimer = millis();

                        // if (catches >= 1 && micros()/MICROS_TO_SEC - lastCatch >= MAX_SEARCH_WAIT_AFTER_ONE - (catches-1)*(GAME_BALL_WAIT_TIME_PENALTY)) {
                        //   catches = TOTAL_ATTEMPTS;
                        //   mode = goalSearch;
                        //   goalYawDirection = searchDirection();  //randomize search direction
                        //   break;
                        // }

                        // if (catches >= TOTAL_ATTEMPTS) {
                        //   mode = goalSearch;
                        //   goalYawDirection = searchDirection();  //randomize search direction
                        // }

                        //move toward the balloon
                        yawCom = xPID.calculate(GAME_BaLL_X_OFFSET, tx, dt/1000); 
                        upCom = -yPID.calculate(GAME_BALL_APPROACH_ANGLE, ty, dt/1000);  
                        forwardCom = GAME_BALL_CLOSURE_COM;
                        translationCom = 0;

                        //check if the gate should be opened
                        if (tz < BALL_GATE_OPEN_TRIGGER) {
                            ballGrabber.openGrabber(blimp_state);

                            //check if the catching mode should be triggered
                            if (tz < BALL_CATCH_TRIGGER) {
                                auto_state = catching;

                                //start catching timer
                                catchTimeStart = millis();

                                //turn motors off
                                motorControl.update(0,0,0,0,0);
                            }
                        } else {
                            //make sure grabber is closed, no game ball is close enough to catch
                            ballGrabber.closeGrabber(blimp_state);
                        }

                        //if target is lost within 1 second
                        //remember the previous info about where the ball is 
                    }
                    else if((millis()-catchMemoryTimer) < 1000 && detected_target.size() == 0){
                            yawCom = xPID.calculate(GAME_BaLL_X_OFFSET, temp_tx, dt/1000); 
                            upCom = -yPID.calculate(GAME_BALL_APPROACH_ANGLE, temp_ty, dt/1000);  
                            forwardCom = GAME_BALL_CLOSURE_COM;
                            translationCom = 0;
                    } 
                    // after two seconds of losing the target, the target is still not detected
                    else {
                        //no target, look for another
                        //maybe add some memory
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                        ballGrabber.closeGrabber(blimp_state);
                        searchYawDirection = searchDirection();  //randomize the search direction
                    }

                    break;
                } case catching: {
                    //wait for 0.3 second
                    // delay(300);

                    forwardCom = CATCHING_FORWARD_COM;
                    upCom = -CATCHING_UP_COM;
                    yawCom = 0;
                    translationCom = 0;

                    if (catchTimeStart < millis() - catchTime) {
                        //catching ended, start caught timer
                        auto_state = caught;
                        caughtTimeStart = millis();
                        ballGrabber.closeGrabber(blimp_state);

                        //increment number of catches
                        catches = catches + 1;

                        //start catch timmer
                        lastCatch = micros()/MICROS_TO_SEC;
                    }
                    break;
                } case caught: {
                    if (catches > 0) {
                        //if a target is seen right after the catch
                        if (detected_target.size() > 0) {
                            //approach next game ball if visible
                            if (catches < TOTAL_ATTEMPTS) {
                                auto_state = searching;
                                // searching timer
                                searchingTimeStart = millis();
                                searchYawDirection = searchDirection();  //randomize the search direction
                            }
                        }

                        //decide if the blimp is going to game ball search or goal search
                        if (caughtTimeStart < millis() - caughtTime) {
                            if (catches >= TOTAL_ATTEMPTS) {
                                auto_state = goalSearch;
                                goalYawDirection = searchDirection();  //randomize search direction
                            } else {
                                auto_state = searching;
                                // searching timer
                                searchingTimeStart = millis();
                                searchYawDirection = searchDirection();  //randomize the search direction
                            }
                        }

                        forwardCom = CAUGHT_FORWARD_COM;
                        upCom = -CAUGHT_UP_COM;
                        yawCom = 0;
                    } else {
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                        searchYawDirection = searchDirection();  //randomize the search direction
                    }
                    break;
                } case goalSearch: {
                    if (catches >= TOTAL_ATTEMPTS) {

                        //keep ball grabber closed
                        // ballGrabber.closeGrabber();

                        //use object avoidence
                        double avoidanceMinVal = 1000.0; // Initialize 
                        int avoidanceMinIndex = 10;

                        // Iterate through the vector to find the minimum value and its index
                        // find the minimum distance and its corresponding quadrant number (1-9)
                        for (int i = 0; i < 9; ++i) {
                            if (avoidance[i] < avoidanceMinVal) {
                                avoidanceMinVal = avoidance[i]; //distance
                                avoidanceMinIndex = i+1; //quadrant number
                            }
                        }

                        //set the avoidance quadrant only in range
                        if (avoidanceMinVal < AVOID_TRIGGER) {
                            //update quadrant
                            quadrant = avoidanceMinIndex;
                        } else {
                            //update quadrant
                            //safe
                            quadrant = 10;
                        }

                        if (quadrant != 10 && USE_OBJECT_AVOIDENCE) {
                            //avoding obstacle
                            //overide search commands
                            yawCom = yawA;
                            forwardCom = forwardA;
                            upCom = upA;
                        } else {
                            //goal search behavior
                            //randomize the diretion selection
                            yawCom = GOAL_YAW_SEARCH*goalYawDirection;
                            upCom = -goalPositionHold.calculate(GOAL_HEIGHT, actualBaro);  //go up to the goal
                            // upCom = GOAL_UP_VELOCITY;
                            forwardCom = GOAL_FORWARD_SEARCH;
                        }

                        if (detected_target.size() > 0) {
                            auto_state = approachGoal;
                        }
                        
                    } else {
                        auto_state = searching;
                        // searching timer
                        searchingTimeStart = millis();
                        searchYawDirection = searchDirection();  //randomize the search direction
                        ballGrabber.closeGrabber(blimp_state);
                    }
                    break;
                } case approachGoal: {
                    if (detected_target.size() > 0 && catches >= TOTAL_ATTEMPTS) {
                        yawCom = xPID.calculate(GOAL_X_OFFSET, tx, dt);
                        upCom = -yPID.calculate(GOAL_APPROACH_ANGLE, ty, dt);
                        forwardCom = GOAL_CLOSURE_COM;

                        if ((tz < GOAL_DISTANCE_TRIGGER && goalColor == orange) || (tz < GOAL_DISTANCE_TRIGGER && goalColor == yellow)) {
                            scoreTimeStart = millis();
                            auto_state = scoringStart;
                        }
                    } else {
                        auto_state = goalSearch;
                        goalYawDirection = searchDirection();  //randomize search direction
                        ballGrabber.closeGrabber(blimp_state);
                    }
                    break;
                } case scoringStart: {
                    //after correction, we can do goal alignment with a yaw and a translation 
                    if (true) {
                        yawCom = SCORING_YAW_COM;
                        forwardCom = SCORING_FORWARD_COM;
                        upCom = -SCORING_UP_COM;

                        if (scoreTimeStart < millis() - scoreTime) {
                            auto_state = shooting;     
                            shootingTimeStart = millis();
                            break;
                        }
                    }
                    break;
                } case shooting: {
                    if (true) {
                        yawCom = 0;
                        forwardCom = SHOOTING_FORWARD_COM;
                        upCom = -SHOOTING_UP_COM;

                        ballGrabber.shoot(blimp_state);
                        catches = 0;

                        if (shootingTimeStart < millis() - shootingTime) {
                            ballGrabber.closeGrabber(blimp_state);
                            scoredTimeStart = millis();
                            auto_state = scored;
                            break;
                        }
                    }
                    break;
                } case scored: {
                    if (true) {

                        // ballGrabber.closeGrabber(blimp_state);

                        yawCom = 0;
                        forwardCom = SCORED_FORWARD_COM;
                        upCom = SCORED_UP_COM;

                        if (scoredTimeStart < millis() - scoredTime) {
                            auto_state = searching;
                            // searching timer
                            searchingTimeStart = millis();
                            searchYawDirection = searchDirection();  //randomize the search direction
                            break;
                        }
                    }
                    break;
                } default: {
                    //shouldn't get here
                    yawCom = 0;
                    forwardCom = 0;
                    upCom = 0;
                    break;
                }
            } //End switch
        } else {
            //Blimp is lost
            forwardCom = 0.0;
            upCom = 0.0;
            yawCom = 0.0;
        }

        //publish blimp_state machine info to Basestation
        state_machine_msg.data = auto_state;
        RCSOFTCHECK(rcl_publish(&state_machine_publisher, &state_machine_msg, NULL));

        //safty height 
        // if (actualBaro > MAX_HEIGHT) {
        //     upCom = -1;
        // }
        //translation velocity and command
        // Serial.print(">z v:");
        // Serial.println(-yekf.v);
        // Serial.print(">z com:");
        // Serial.println(translationCom);

        //PID controllers
        float yawMotor = 0.0;
        float upMotor = 0.0;
        float forwardMotor = 0.0;
        float translationMotor = 0.0;

        //hyperbolic tan for yaw "filtering"
        float deadband = 5; // deadband for filteration
        yawMotor = yawPID.calculate(yawCom, yawRateFilter.last, dt);  

        if (abs(yawCom-yawRateFilter.last) < deadband) {
           
            yawMotor = 0;

        } else {

            yawMotor = tanh(yawMotor)*abs(yawMotor);

        }

        //TO DO: improve velocity control
        // upMotor = verticalPID.calculate(upCom, kf.v, dt); //up velocity from barometer
        // What's up motor? :)
        upMotor = upCom;

        if (USE_EST_VELOCITY_IN_MANUAL == true) {
            //using kalman filters for the current velosity feedback for full-blimp_state feeback PID controllers

            // forwardMotor = forwardPID.calculate(forwardCom, xekf.v, dt);  //extended filter
            // float forwardMotor = forwardPID.calculate(forwardCom, kal_vel.x_vel_est, dt);
            // translationMotor = translationPID.calculate(translationCom, yekf.v, dt); //extended filter
            // float translationMotor = translationPID.calculate(translationCom, kal_vel.y_vel_est, dt); 
        } else{
            //without PID
            forwardMotor = forwardCom;
            translationMotor = translationCom;
        }

        //motor debug
        debug_msg.data.data[0] = yawCom;
        debug_msg.data.data[1] = upCom;
        debug_msg.data.data[2] = translationCom;
        debug_msg.data.data[3] = forwardCom;
        debug_msg.data.size = 4;

        // debug_msg.data.data[0] = forward_msg;
        // debug_msg.data.data[1] = yaw_msg;
        // debug_msg.data.data[2] = up_msg;
        // debug_msg.data.data[3] = translation_msg;
        // debug_msg.data.size = 4;

        //test target messages
        // debug_msg.data.data[0] = targets[0];
        // debug_msg.data.data[1] = targets[1];
        // debug_msg.data.data[2] = targets[2];
        // debug_msg.data.data[3] = targets[3];
        // debug_msg.data.data[4] = targets[4];
        // debug_msg.data.data[5] = targets[5];
        // debug_msg.data.data[6] = targets[6];
        // debug_msg.data.data[7] = targets[7];
        // debug_msg.data.data[8] = targets[8];
        // debug_msg.data.size = 9;

        RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

        //Serial.print(">up current:");
        //Serial.println(kf.v);
        //float yawCurrent =  (float)yawRateFilter.last;
        //Serial.print(">yaw current");
        //Serial.println(yawCurrent);
        //Serial.print(">forward current:");
        //Serial.println(xekf.v);
        //Serial.print(">translation current:");
        //Serial.println(yekf.v);

        //gimbal + motor updates
        ballGrabber.update();

        if (loop_time < 10 + firstMessageTime) {
            //filter base station data
            baroOffset.filter(baseBaro-BerryIMU.comp_press);
            rollOffset.filter(BerryIMU.gyr_rateXraw);

            //zero motors while filters converge and esc arms
            motorControl.update(0, 0, 0, 0, 0);
            bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
            bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight);
            leftGimbal.updateGimbal(leftReady && rightReady);
            rightGimbal.updateGimbal(leftReady && rightReady);
        } else {
            if (blimp_state == manual && !MOTORS_OFF) {
                //forward, translation, up, yaw, roll
                if (!ZERO_MODE) motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upLeft, motorControl.forwardLeft);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, motorControl.upRight, motorControl.forwardRight);
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            } else if (blimp_state == autonomous && !MOTORS_OFF) {
                motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight); 
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            } else if(MOTORS_OFF){
                motorControl.update(forwardMotor, -translationMotor, upMotor, yawMotor, 0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawLeft, motorControl.upLeft, motorControl.forwardLeft);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, motorControl.yawRight, motorControl.upRight, motorControl.forwardRight); 
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            } else {
                motorControl.update(0,0,0,0,0);
                bool leftReady = leftGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
                bool rightReady = rightGimbal.readyGimbal(GIMBAL_DEBUG, MOTORS_OFF, 0, 0, 0, 0, 0);
                leftGimbal.updateGimbal(leftReady && rightReady);
                rightGimbal.updateGimbal(leftReady && rightReady);
            }
        }
    }
}
