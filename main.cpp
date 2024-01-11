#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <Servo.h>
#include <DFRobot_QMC5883.h>
#include <SensorFusion.h>

Adafruit_MPU6050 mpu;
SF fusion;
DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

//PID control
float K_P = 0.6;
float K_I = 0.5;
float K_D = 0.14;
float sensor_offset = 15;

int counter = 1;

int rotary_encoder = PB14;

//magnetometer calibration
float b [3] = {-70.322132, -234.665158, 271.251127};
float A [3][3] = {{0.032394, -0.000213, 0.000050},
                  {-0.000213, 0.031874, 0.000346},
                  {0.000050, 0.000346, 0.032744}};

float deltat;
float AccX, AccY, AccZ;
float roll, pitch;
float yaw = 180;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float previousTime;
float position [2];
float position_front [2];
float position_sensor[2];
float orientation_past [2];
float orientation [2];
float errors [2]; //[current error, past error]
uint32_t errors_time [2]; // [current_error_time, ..., past_error_time]
float rotary_encoder_distance = (320.0-3.0) / (11.0*5.0);
float e_i = 0;
uint32_t past_time = millis();
int servo_val = 127;
bool onBridge = false;
bool passedBridge = false;
bool passedTurn = false;
float distance_threshold = 80;
float yaw_offset = 0;

//defining speed and differential stearing for each section of the map
float velocity_order [8] = {0.2, 0.3, 0.3, 0.3, 0.9, 0.8, 1, 0.9};
float offset_order [8] = {0.15, 0.2, 0.2, 0.2, -0.7, 0.2, 0, -0.6};

//defining the map
float lines[5][4] = {{156, 54, 110, 250},
                     {110, 250, 100, 300},
                     {85, 265, 85, 415}, //{78, 300, 78, 600},//410},
                     {30.5, 415, 30.5, 255},
                     {30.5, 255, 45, 200}};

float circles[3][10] = {{223, 83, 73, cos(3.55), sin(3.55), cos(-1.52), sin(-1.52), 4.763-3.55},
                         {90-50, 415, 50, cos(-0.021), sin(-0.021), cos(3.0502), sin(3.0502), 3.0502 + 0.021},
                         {83, 193, 31.8, cos(-2.7), sin(-2.7), cos(-0.16), sin(-0.16), 2.7 - 0.16}};

signed char order [2][8] = {{0, -1, -1, -1, 1, -1, -1, 2},
                            {-1, 0, 1, 2, -1, 3, 4, -1}};

int order_index = 0;
float velocity = 0.2;

bool rotary_val_past = 0;

int servoPin = PA1;
Servo myServo;

PinName H_bridge [4] = {PB_8, PB_9, PA_6, PA_7};

void update_error();
float getErrorToLine(float a[], float b[]);
float getErrorToSemiCircle(float centre[], float radius, float ca_1_unit[], float ca_2_unit[], float theta);
void getNextPosition();
void update_PID();
bool on_ramp();
void update_front_pos();
void run_motors(float offset);
void calibration_line(bool y, float m, float b, float horizontal, float vertical);
float magnitude(float array[], int length);

void getOrientation()
{
  sensors_event_t a,g,t;
  while (!mpu.getEvent(&a, &g, &t));
  
  AccX = a.acceleration.x - AccErrorX;
  AccY = a.acceleration.y - AccErrorY;
  AccZ = a.acceleration.z - AccErrorZ;

  float GyroX = g.gyro.x - GyroErrorX;
  float GyroY = g.gyro.y - GyroErrorY;
  float GyroZ = g.gyro.z - GyroErrorZ;

  sVector_t mag = compass.readRaw();

  float mag_hardiron [3] = {mag.XAxis - b[0], mag.YAxis - b[1], mag.ZAxis - b[2]};
  float mag_cal [3] = {0,0,0};
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      mag_cal[i] += A[i][j] * mag_hardiron[j];
    }
  }

  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, mag_cal[0], mag_cal[1], mag_cal[2], deltat);
  yaw = fusion.getYaw() + yaw_offset;
  roll = fusion.getPitch();
  pitch = fusion.getRoll() + 180;
}

void calculateIMUErrors()
{
  float sumAccX = 0;
  float sumAccY = 0;
  float sumAccZ = 0;
  float sumGyroX = 0;
  float sumGyroY = 0;
  float sumGyroZ = 0;
  for (int i=0; i < 3000; i++)
  {
    sensors_event_t a,g,t;
    mpu.getEvent(&a, &g, &t);

    sumAccX += a.acceleration.x;
    sumAccY += a.acceleration.y;
    sumAccZ += a.acceleration.z;

    sumGyroX += g.gyro.x;
    sumGyroY += g.gyro.y;
    sumGyroZ += g.gyro.z;

    delay(1);
  }

  AccErrorX = sumAccX / 3000.0;
  AccErrorY = sumAccY / 3000.0;
  AccErrorZ = sumAccZ / 3000.0 + 9.807;
  GyroErrorX = sumGyroX / 3000.0;
  GyroErrorY = sumGyroY / 3000.0;
  GyroErrorZ = sumGyroZ / 3000.0;
}

void setup() {
  // Serial3.begin(9600);
  // Serial3.println("starting");

  myServo.attach(servoPin);
  pinMode(servoPin,OUTPUT);
  pinMode(PA9, OUTPUT);
  pinMode(PA10, INPUT);

  for (int i = 0; i < 4; i++)
  {
    pinMode(H_bridge[i], OUTPUT);
  }

  position[0] = 244.0 - 4.5;
  position[1] = 2;

  orientation_past[0] = -1;
  orientation_past[1] = 0;

  orientation[0] = -1;
  orientation[1] = 0;

  update_front_pos();

  position_sensor[0] = position_front[0] + sensor_offset * orientation[0];
  position_sensor[1] = position_front[1] + sensor_offset * orientation[1];

  uint32_t time = millis();
  for (int i = 0; i < 2; i++)
  {
    errors_time[i] = time;
  }

  mpu.begin(0x68);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  compass.begin();


  pinMode(rotary_encoder, INPUT);

  calculateIMUErrors();

  fusion.beta = 37.0f;

  for (int i = 0; i < 1000; i++)
  {
    getOrientation();
  }

  velocity = 0.3;
  run_motors(0.2);

  yaw_offset += 180 - yaw;
  getOrientation();
}

float dot_prod(float first[], float second[], int length)
{
  float sum = 0;
  for (int i = 0; i < length; i++)
  {
    sum += first[i] * second[i];
  }
  return sum;
}

float magnitude(float array[], int length)
{
  return sqrt(dot_prod(array, array, length));
}

void calibration_line(bool y, float m, float b, float horizontal, float vertical)
{
  float rotary_fraction = rotary_encoder_distance *  fmin(fabs((millis() - errors_time[0]) / (errors_time[0] - errors_time[1])), 1);
  float t = orientation[0] * m - orientation[1];
  float f = m*orientation[1] + orientation[0];
  float p = m;
  float k = 1;
  if (!y)
  {
    t = orientation[1] / m - orientation[0];
    f = -orientation[0] / m - orientation[1];
    p = 1/m;
    k = -p;
  }
  position[y] = p * position[!y] + t * rotary_fraction + t * vertical + f * horizontal + b * k;
  update_front_pos();
}

void getNextPosition()
{ 
  if (order_index == 2 || order_index == 3)
  {
    if ((millis() - errors_time[0]) * 1.44 < errors_time[0] - errors_time[1])
    {
      return;
    }
  }

  if (order_index == 0)
  {
    K_P = 1.2;
    K_I = 0.7;
  }
  else
  {
    K_P = 0.8;
    K_I = 0.5; //0.1;
    K_D = 0.3; //0.15;
  }

  orientation[0] = cos(yaw * PI / 180.0);
  orientation[1] = sin(yaw * PI / 180.0);
  
  float cos_theta = dot_prod(orientation, orientation_past, 2) 
                    / (magnitude(orientation, 2) * magnitude(orientation_past, 2));
  float theta;
  if (cos_theta >= 1)
  {
    theta = 0;
  }
  else if (cos_theta <= -1)
  {
    theta = PI;
  }
  else
  {
    theta = acos(cos_theta);
  }
  float abs_sin_theta = abs(sin(theta));

  if (theta == 0)
  {
      for (int i = 0; i < 2; i++)
      {
        position[i] += orientation[i] * rotary_encoder_distance;
        position_front[i] += orientation[i] * rotary_encoder_distance;
        position_sensor[i] = position_front[i] + sensor_offset * orientation[i];
      }
  }
  else{
    float r = rotary_encoder_distance / abs(theta);
    float side_val = orientation[1] * orientation_past[0] - orientation[0] * orientation_past[1];
    
    float n [2] = {orientation_past[1], -orientation_past[0]};
    float side_n = orientation_past[0] * n[1] - orientation_past[1] * n[0];

    float mag_n = magnitude(n, 2);
    float n_hat [2] = {n[0] / mag_n, n[1] / mag_n};

    if (signbit(side_val) != signbit(side_n))
    {
      n_hat[0] *= -1;
      n_hat[1] *= -1;
    }
    if (side_val < 0)
    {
      abs_sin_theta *= -1;
    }

    float distance_rotated [2] = {r * (-cos_theta * n_hat[0] + abs_sin_theta * n_hat[1]), 
                                r * (-abs_sin_theta * n_hat[0] - cos_theta * n_hat[1])};

    position[0] += n_hat[0] * r + distance_rotated[0];
    position[1] += n_hat[1] * r + distance_rotated[1];

    update_front_pos(); 
    for (int i = 0; i < 2; i++)
    {
      position_sensor[i] = position_front[i] + sensor_offset * orientation[i];
      orientation_past[i] = orientation[i];
    }
  }
  
  if ((order_index == 4 && !passedTurn) || position_sensor[1] > 425)
  {
    if (abs(orientation[1] + 1) < 0.15)
    {
      passedTurn = true;
      update_error();
      update_PID();
    }
    else 
      myServo.write(169);
  }
  else
  {
    update_error();
    update_PID();
  }

  velocity = velocity_order[order_index];
  run_motors(offset_order[order_index]);
  
  if ((order_index == 2 || order_index == 3) && passedBridge && position[1] > 264.5 && position[1] < 357)
  {
    float sensor_val = analogRead(PA4);
    float distance = 3727.5937 / pow(sensor_val, 0.892857) - 1.6;
    if (distance < 80)
      calibration_line(false, 1000.0, 250.0 - 1000.0 * (61 + distance * abs(orientation[1])), 1.7, 7.6);
  }
}

void update_PID()
{
  float e_p = errors[0];
  if (signbit(e_p) == signbit(e_i))
    e_i += (errors_time[0] - errors_time[1]) * errors[1] / 1000.0;
  else
    e_i = 0;

  float e_d = (errors[0] - errors[1]) * 1000.0 / (errors_time[0] - errors_time[1]);

  float turning = - e_p * K_P - e_d * K_D - K_I * e_i;
  servo_val = 127;
  if (turning != 0)
  {
    float sig = 2.0*(1.0/(1+pow(1.1, -turning)) - 0.5);
    servo_val = (int)((((turning > 0) * (169.0-127.0) + (turning < 0) * (127.0-69.0))*sig + 127.0) / 3) * 3;
  }

  myServo.write(servo_val);
}

void update_error()
{
  if (order_index == 2 && position_sensor[1] > 300)
  {
    order_index = 3;
    K_D = 0;
  }

  float min_distance = 1000;
  float final_error = 0;
  int new_order_index = order_index;
  int i = order_index;
  float errors_new = 10000;
  for (int index = 0; index < 2; index++)
  {
    if (order[0][i] != -1) //then it is a circle
    {
      float centre[2] = {circles[order[0][i]][0], circles[order[0][i]][1]};
      float ca_1_unit[2] = {circles[order[0][i]][3], circles[order[0][i]][4]};
      float ca_2_unit[2] = {circles[order[0][i]][5], circles[order[0][i]][6]};

      errors_new = getErrorToSemiCircle(centre, circles[order[0][i]][2], ca_1_unit, ca_2_unit, circles[order[0][i]][7]);
    }
    else //it is a line
    {
      float a[2] = {lines[order[1][i]][0], lines[order[1][i]][1]};
      float b[2] = {lines[order[1][i]][2], lines[order[1][i]][3]};
      errors_new = getErrorToLine(a, b);
    }
    if (abs(errors_new) < min_distance)
    {
      min_distance = abs(errors_new);
      final_error = errors_new;
      new_order_index = i;
    }
    i += 1;
    int size = (int)(sizeof(order[0]) / sizeof(order[0][0]));
    if (i ==  size - 1)
    {
      break;
    }
    if (i == size)
    {
      i = 2;
    }
  }
  order_index = new_order_index;

  errors[1] = errors[0];
  errors_time[1] = errors_time[0];

  errors[0] = final_error;
  errors_time[0] = millis();
}

float getErrorToLine(float a[], float b[])
{
  float ab [2] = {b[0] - a[0], b[1] - a[1]};
  float l2 = pow(ab[0], 2) + pow(ab[1], 2);
  float ap [2] = {position_sensor[0] - a[0], position_sensor[1] - a[1]};
  
  float t = fmax(0, fmin(1, dot_prod(ap, ab, 2) / l2));
  float shortest_length [2] = {position_sensor[0] - a[0] - t * ab[0], position_sensor[1] - a[1] - t*ab[1]};
  float abs_error = magnitude(shortest_length, 2);
  
  float compare = orientation[0] * shortest_length[1] - orientation[1] * shortest_length[0];
  int compare_sign = ((compare > 0) - (compare < 0));
  return abs_error * compare_sign;
}

float getErrorToSemiCircle(float centre[], float radius, float ca_1_unit[], float ca_2_unit[], float theta)
{
  float cp [2] = {position_sensor[0] - centre[0], position_sensor[1] - centre[1]};
  float cp_length = magnitude(cp, 2);
  float cp_unit [2] = {cp[0] / cp_length, cp[1] / cp_length};

  bool passed = false;
  int filter1 = floor(sqrt(2)/2.0 * (ca_1_unit[0]*cp_unit[1] - ca_1_unit[1]*cp_unit[0]));
  int filter2 = floor(sqrt(2)/2.0 * (ca_2_unit[1]*cp_unit[1] + ca_2_unit[0]*cp_unit[0]))
                    + floor(sqrt(2)/2.0 * (ca_2_unit[1] * cp_unit[0] - ca_2_unit[0] * cp_unit[1]));
  if (theta <= PI)
  {
    filter1 += floor(sqrt(2)/2.0 * (ca_1_unit[0] * cp_unit[0] + ca_1_unit[1] * cp_unit[1]));
    if ((theta < PI/2 && filter1 == 0 && filter2 == 0) || (theta >= PI/2 && (filter1 == 0 || filter2 == 0)))
      passed = true;
  }
  else if (filter1 == 0 || filter2 == 0)
  {
      passed = true;
  }
  
  signed char sign = 1;
  if (passed)
  {
    float error = cp_length - radius;
    float dot = cp[1] * orientation[0] - cp[0] * orientation[1];
    
    if (!((error > 0 && dot > 0) || (error < 0 && dot < 0)))
    {
      sign = -1;
    }

    return abs(error) * sign;
  }
  
  float distance1 [2] = {centre[0] + ca_1_unit[0] * radius - position_sensor[0], centre[1] + ca_1_unit[1] * radius - position_sensor[1]};
  float distance2 [2] = {centre[0] + ca_2_unit[0] * radius - position_sensor[0], centre[1] + ca_2_unit[1] * radius - position_sensor[1]};

  float norm_d1 = dot_prod(distance1, distance1, 2);
  float norm_d2 = dot_prod(distance2, distance2, 2);

  float error = sqrt(fmin(norm_d1, norm_d2));
  if (norm_d1 < norm_d2)
      sign = -orientation[0] * distance1[1]  + orientation[1] * distance1[0];
  else
      sign = -orientation[0] * distance2[1] + orientation[1] * distance2[0];
  if (sign != 0)
    return abs(error) * sign / abs(sign);
  return abs(error) * -1;
}

void update_front_pos()
{
  position_front[0] = position[0] + 17.4 * orientation[0] + 10.8 * orientation[1];
  position_front[1] = position[1] + 17.4 * orientation[1] - 10.8 * orientation[0];
}

void run_motors(float offset)
{
  pwm_start(H_bridge[1], 250, (velocity + offset) * 4096, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(H_bridge[3], 250, velocity * 4096, RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(H_bridge[1], 250, 0, RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(H_bridge[3], 250, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(H_bridge[0], 250, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(H_bridge[2], 250, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

void loop() {
  getOrientation();
  
  bool rotary_val = digitalRead(rotary_encoder);
  if ((rotary_val != rotary_val_past) && rotary_val == 0 && millis() - past_time > 20)
  {
    past_time = millis();
    //Serial3.println("rotary hit");
    getNextPosition();
  }
  rotary_val_past = rotary_val;

  if ((order_index == 1 || order_index == 2 || order_index == 7) && !passedBridge && position[1] > 150)
  {
    digitalWrite(PA9, HIGH);
    delayMicroseconds(4);

    digitalWrite(PA9, LOW);
    delayMicroseconds(4);
    if (pulseIn(PA10, HIGH) < 3000)
    {
      calibration_line(true, 0, 224.5, 16.1, 11.4);
      passedBridge = true;
    }
  }
  else if (order_index == 5 || order_index == 6)
  {
    if (AccX*AccX + AccY*AccY + AccZ*AccZ < 6)
    {
      float cliff_vel = rotary_encoder_distance / (errors_time[0] - errors_time[1]);
      order_index = 7;
      velocity = 0;
      run_motors(0);
      uint32_t time1 = micros();
      while (AccX*AccX + AccY*AccY + AccZ*AccZ < 6)
      {
        getOrientation();
      }
      uint32_t time2 = micros();
      float delta_d = cliff_vel * (time2 - time1) / 1000.0;
      position[0] = position[0] + orientation[0] * delta_d;
      position[1] = 224.5 + orientation[1] * delta_d;
      update_front_pos();
      for (int i = 0; i < 400; i++)
      {
        getOrientation();
      }

      passedBridge = false;
      passedTurn = false;
      getNextPosition();
    }
  }
}
