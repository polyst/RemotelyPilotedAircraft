#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Servo.h>
#include <PID_v1.h>
#include <FS.h>

#define LEFT_MOTOR_PIN 12
#define RIGHT_MOTOR_PIN 14
#define YAW_PIN 0
#define PITCH_PIN 15
#define ROLL_PIN 13

#define INTERRUPT_PIN 0  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

String system_info_file = "/system_info.json"; 

uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t rc_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

double yaw = 0;
double power = 0;
double diff_power = 0;
double led = false;
double roll = 0;
double pitch = 0;
bool flight_control = false;
double flap_angle = 0;
bool landing_gear = false;

int yaw_correct = 0;
int twin_engine_power_correct = 0;
int roll_correct = 0;
int pitch_correct = 0;

double mpu_yaw = 0;
double mpu_roll = 0;
double mpu_pitch = 0;

MPU6050 mpu;
int t;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

double kp = 4;
double ki = 4;
double kd = 0.25;
double roll_power_out = 0;

PID rollPowerPID(&mpu_roll, &roll_power_out, &roll,kp,ki,kd, DIRECT);

Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll;

void save_cofig(){
  DynamicJsonDocument system_info(512);
  system_info["kp"] = kp;
  system_info["ki"] = ki;
  system_info["kd"] = kd;
  system_info["twin_engine_power_correct"] = twin_engine_power_correct;
  system_info["yaw_correct"] = yaw_correct;
  system_info["roll_correct"] = roll_correct;
  system_info["pitch_correct"] = pitch_correct;
  String system_info_str;
  serializeJson(system_info, system_info_str);
  Serial.println(system_info_str);
  File file = SPIFFS.open(system_info_file,"w");
  serializeJson(system_info, file);
  file.close();
}

void OnDataReceived(uint8_t *macAddr, uint8_t *data, uint8_t dataLength) {
  String receivedText;
  char text[dataLength+1];
  memcpy(text, (char *)data, dataLength);
  text[dataLength] = '\0';
  receivedText = String(text);
  //Serial.println(receivedText);
  DynamicJsonDocument order_data(512);
  deserializeJson(order_data, receivedText);
  if(order_data["from"] == "rc"){
    if(areMacAddressesEqual(rc_address,broadcast_address)){
      if(order_data["request"] == "matching_codes"){
        for(int i = 0;i < 6 ; i++){
          rc_address[i] = macAddr[i];
        }
        Serial.println("RC MATCHING CODES");
      }
    }
    if(areMacAddressesEqual(rc_address,macAddr)){
      if(order_data["request"] == "flight_order"){
        yaw = order_data["yaw"];
        power = order_data["power"];
        diff_power = order_data["diff_power"];
        roll = order_data["roll"];
        pitch = order_data["pitch"];
        flight_control = order_data["flight_control"];
        flap_angle = order_data["flap_angle"];
        landing_gear = order_data["landing_gear"];
        led = order_data["led"];
      }else{
        Serial.println(receivedText);
      }
      if(order_data["request"] == "yaw_left"){
        yaw_correct = yaw_correct - 1;
        save_cofig();
      }
      if(order_data["request"] == "yaw_right"){
        yaw_correct = yaw_correct + 1;
        save_cofig();
      }
      if(order_data["request"] == "twin_engine_left"){
        twin_engine_power_correct = twin_engine_power_correct - 1;
        save_cofig();
      }
      if(order_data["request"] == "twin_engine_right"){
        twin_engine_power_correct = twin_engine_power_correct + 1;
        save_cofig();
      }
      if(order_data["request"] == "roll_left"){
        roll_correct = roll_correct - 1;
        save_cofig();
      }
      if(order_data["request"] == "roll_right"){
        roll_correct = roll_correct + 1;
        save_cofig();
      }
      if(order_data["request"] == "pitch_up"){
        pitch_correct = pitch_correct + 1;
        save_cofig();
      }
      if(order_data["request"] == "pitch_down"){
        pitch_correct = pitch_correct - 1;
        save_cofig();
      }
      if(order_data["request"] == "kp_down"){
        kp = kp - 0.01;
        rollPowerPID.SetTunings(kp, ki, kd);
        save_cofig();
      }
      if(order_data["request"] == "kp_up"){
        kp = kp + 0.01;
        rollPowerPID.SetTunings(kp, ki, kd);
        save_cofig();
      }
      if(order_data["request"] == "ki_down"){
        ki = ki - 0.01;
        rollPowerPID.SetTunings(kp, ki, kd);
        save_cofig();
      }
      if(order_data["request"] == "ki_up"){
        ki = ki + 0.01;
        rollPowerPID.SetTunings(kp, ki, kd);
        save_cofig();
      }
      if(order_data["request"] == "kd_down"){
        kd = kd - 0.01;
        rollPowerPID.SetTunings(kp, ki, kd);
        save_cofig();
      }
      if(order_data["request"] == "kd_up"){
        kd = kd + 0.01;
        rollPowerPID.SetTunings(kp, ki, kd);
        save_cofig();
      }
    }
  }
}

bool areMacAddressesEqual(const uint8_t mac1[6], const uint8_t mac2[6]) {
    for (int i = 0; i < 6; i++) {
        if (mac1[i] != mac2[i]) {
            return false;
        }
    }
    return true;
}

void setup() {
  Serial.begin(115200);   // 初始化串口
  while (!Serial);   
  while (!SPIFFS.begin());
  Wire.begin();
  Wire.setClock(400000);

  delay(1000);

  if(SPIFFS.exists(system_info_file)){            // 判断配置文件是否存在
    Serial.println("SYSTEM INFO Loading!");
    DynamicJsonDocument system_info(512);
    File file = SPIFFS.open(system_info_file,"r");   // 打开配置文件
    deserializeJson(system_info, file);
    String system_info_str;
    serializeJson(system_info, system_info_str);
    Serial.println(system_info_str);
    file.close();
    kp = system_info["kp"];
    ki = system_info["ki"];
    kd = system_info["kd"];
    twin_engine_power_correct = system_info["twin_engine_power_correct"];
    yaw_correct = system_info["yaw_correct"];
    roll_correct = system_info["roll_correct"];
    pitch_correct = system_info["pitch_correct"];
  }else{
    Serial.println("SYSTEM INFO Create!");
    DynamicJsonDocument system_info(512);
    kp = 1.6;
    ki = 0;
    kd = 350;
    yaw_correct = 0;
    roll_correct = 0;
    pitch_correct = 0;
    twin_engine_power_correct = 0;
    system_info["kp"] = kp;
    system_info["ki"] = ki;
    system_info["kd"] = kd;
    system_info["twin_engine_power_correct"] = twin_engine_power_correct;
    system_info["yaw_correct"] = yaw_correct;
    system_info["roll_correct"] = roll_correct;
    system_info["pitch_correct"] = pitch_correct;
    String system_info_str;
    serializeJson(system_info, system_info_str);
    Serial.println(system_info_str);
    File file = SPIFFS.open(system_info_file,"w");
    serializeJson(system_info, file);
    file.close();
  }

  WiFi.mode(WIFI_STA);

  if (esp_now_init() == 0) {
    Serial.println("ESPNow Init Success");
  }else{
    Serial.println("ESPNow Init Failed");
    // ESP-NOW 初始化失败重启系统
    ESP.restart();
  }

  // 设置 接收机为主控模式
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER); 
  esp_now_register_recv_cb(OnDataReceived);

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      //Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      //enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  servo_yaw.attach(0);
  servo_pitch.attach(13);
  servo_roll.attach(15);
  
  servo_roll.write(90);
  servo_pitch.write(90);
  servo_yaw.write(90);

  rollPowerPID.SetOutputLimits(-100, 100);
  rollPowerPID.SetMode(AUTOMATIC);  

  analogWrite(2, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(dmpReady){
    mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    // 获取 YAW PITCH ROLL 硬解数据
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mpu_yaw = ypr[0] * 180/M_PI;
    mpu_pitch = ypr[1] * 180/M_PI;
    mpu_roll  = -ypr[2] * 180/M_PI;

    //Serial.print("YAW : ");
    //Serial.print(mpu_yaw);
    //Serial.print("; PITCH : ");
    //Serial.print(mpu_pitch);
    //Serial.print("; ROLL : ");
    //Serial.print(mpu_roll);
    //Serial.print("; RC ROLL : ");
    //Serial.print(roll);
    //Serial.print("; FC : ");
    //Serial.print(flight_control);
    //Serial.print("; ");
      
    // 获取姿态数据
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //plane_order["ax"] = aaReal.x;
    //plane_order["ay"] = aaReal.y;
    //plane_order["az"] = aaReal.z;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    //plane_order["gx"] = gravity.x;
    //plane_order["gy"] = gravity.y;
    //plane_order["gz"] = gravity.z; 
  }

  if(areMacAddressesEqual(rc_address,broadcast_address)){
    DynamicJsonDocument plane_order(512);
    plane_order["request"] = "matching_codes";
    plane_order["from"] = "plane";
    String plane_order_str;
    serializeJson(plane_order, plane_order_str);
    char order_data[plane_order_str.length() + 1];
    strcpy(order_data, plane_order_str.c_str());
    esp_now_send(rc_address, (uint8_t *)&order_data, sizeof(order_data));
    //Serial.println(plane_order_str);
    delay(1000);
  }else{
    
    int power_right = 0;
    int power_left = 0;
    if(flight_control){
      if(power > 0){
        if(roll_power_out>0){
          power_right = power - twin_engine_power_correct;
          power_left = power + roll_power_out + twin_engine_power_correct;
        }else{
          power_right = power - roll_power_out - twin_engine_power_correct;
          power_left = power + twin_engine_power_correct;
        }
      }else{
        power_right = 0;
        power_left = 0;
      }
      rollPowerPID.Compute();
      //Serial.print(" roll_power_out : ");
      //Serial.print(roll_power_out);
      //Serial.print("; ");
    }else{
      if(power > 0){
        if(diff_power > 0){
          power_right = power  - twin_engine_power_correct;
          power_left = power + diff_power + twin_engine_power_correct;
        }else{
          power_right = power - diff_power - twin_engine_power_correct;
          power_left = power  + twin_engine_power_correct;
        }
      }
    }
    //Serial.print("POWER RIGHT : ");
    //Serial.print(power_right);
    //Serial.print("; POWER LEFT : ");
    //Serial.print(power_left);
    //Serial.println(";");
    analogWrite(RIGHT_MOTOR_PIN, power_right); 
    analogWrite(LEFT_MOTOR_PIN, power_left);

    servo_yaw.write(90 + yaw + yaw_correct);
    servo_pitch.write(90 - pitch + pitch_correct);
    servo_roll.write(90 + roll + roll_correct);

    if(millis() - t > 100){
      t = millis();
      DynamicJsonDocument plane_order(512);
      plane_order["request"] = "plane_state";
      plane_order["from"] = "plane";
      plane_order["plane_yaw"] = mpu_yaw;
      plane_order["plane_power"] = power;
      plane_order["plane_diff_power_out"] = twin_engine_power_correct;
      plane_order["plane_roll"] = mpu_roll;
      plane_order["plane_pitch"] = mpu_pitch;
      plane_order["plane_pid_kp"] = kp;
      plane_order["plane_pid_ki"] = ki;
      plane_order["plane_pid_kd"] = kd;
      String plane_order_str;
      serializeJson(plane_order, plane_order_str);
      char order_data[plane_order_str.length() + 1];
      strcpy(order_data, plane_order_str.c_str());
      esp_now_send(rc_address, (uint8_t *)&order_data, sizeof(order_data));
    }

    


    delay(10);
  }
}
