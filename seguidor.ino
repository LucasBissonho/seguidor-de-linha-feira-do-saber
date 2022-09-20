// pinos dos sensores
int sensor_ext_left = A0;
int sensor_left = A1;
int sensor_center = A2;
int sensor_right = A3;
int sensor_ext_right = A4;

// pinos dos motores
int motor_r = 4;       // M1 = 4
int motor_r_speed = 5; // E1 = 5
int motor_l = 7;       // M2 = 7
int motor_l_speed = 6; // E2 = 6

int MOTOR_FORWARD = HIGH;
int MOTOR_BACKWARD = LOW;

int state_sensor[5]; // armazena o estado lido dos sensores
int lumus = 350;

int speed_right = 0, speed_left = 0;

int error = 0, last_error = 0;
int proportional = 0, derivative = 0, integral = 0, PID = 0;
int Kp = 300, Ki = 0, Kd = 250;

int speed_A = 255;

//multiplicador do erro
float MULT_ERR = 1.00;

//erro para cada estado
float ERR_EXT_LEFT     =  5 * MULT_ERR;
float ERR_VERY_LEFT    =  3 * MULT_ERR;
float ERR_LEFT         =  2 * MULT_ERR;
float ERR_SLIGHT_LEFT  =  1 * MULT_ERR;
float ERR_FOLLOW       =  0.00 * MULT_ERR;
float ERR_SLIGHT_RIGHT = -1 * MULT_ERR;
float ERR_RIGHT        = -2 * MULT_ERR;
float ERR_VERY_RIGHT   = -3 * MULT_ERR;
float ERR_EXT_RIGHT    = -5 * MULT_ERR;

void setup()
{
  Serial.begin(9600);

  // sensor_setup
  pinMode(sensor_right, INPUT);
  pinMode(sensor_center, INPUT);
  pinMode(sensor_left, INPUT);
  pinMode(sensor_ext_right, INPUT);
  pinMode(sensor_ext_left, INPUT);

  // motor_setup
  pinMode(motor_l, OUTPUT);
  pinMode(motor_r, OUTPUT);
  digitalWrite(motor_l, MOTOR_FORWARD);
  digitalWrite(motor_r, MOTOR_FORWARD);
}

void loop()
{
  readSensor();
  error_calc();
  pid_calc();
  motor_control();
}

void readSensor()
{
  // led do sensor acende quando está na cor branco
  // branco == 0 preto == 1
  state_sensor[0] = analogRead(sensor_ext_left) < lumus ? 0 : 1;
  state_sensor[1] = analogRead(sensor_left) < lumus ? 0 : 1;
  state_sensor[2] = analogRead(sensor_center) < lumus ? 0 : 1;
  state_sensor[3] = analogRead(sensor_right) < lumus ? 0 : 1;
  state_sensor[4] = analogRead(sensor_ext_right) < lumus ? 0 : 1;

  /*
  for (int i = 0; i < 5; i++) {
    Serial.print("sensor ");
    Serial.println(i);
    Serial.println(state_sensor[i]);
  }
  delay(500);
  */
}

int error_calc()
{
  if (state_sensor[0] == 1 &&
      state_sensor[1] == 0 &&
      state_sensor[2] == 0 &&
      state_sensor[3] == 0 &&
      state_sensor[4] == 0) {
    error = ERR_EXT_RIGHT;
  } else if (state_sensor[0] == 1 &&
             state_sensor[1] == 1 &&
             state_sensor[2] == 0 &&
             state_sensor[3] == 0 &&
             state_sensor[4] == 0) {
    error = ERR_VERY_RIGHT;
  } else if (state_sensor[0] == 0 &&
             state_sensor[1] == 1 &&
             state_sensor[2] == 0 &&
             state_sensor[3] == 0 &&
             state_sensor[4] == 0) {
    error = ERR_RIGHT;
  }else if (state_sensor[0] == 0 &&
            state_sensor[1] == 1 &&
            state_sensor[2] == 1 &&
            state_sensor[3] == 0 &&
            state_sensor[4] == 0) {
    error = ERR_SLIGHT_RIGHT;
  }else if (state_sensor[0] == 0 &&
            state_sensor[1] == 0 &&
            state_sensor[2] == 1 &&
            state_sensor[3] == 0 &&
            state_sensor[4] == 0) {
    error = ERR_FOLLOW;
  }else if (state_sensor[0] == 0 &&
            state_sensor[1] == 0 &&
            state_sensor[2] == 1 &&
            state_sensor[3] == 1 &&
            state_sensor[4] == 0) {
    error = ERR_SLIGHT_LEFT;
  }else if (state_sensor[0] == 0 &&
            state_sensor[1] == 0 &&
            state_sensor[2] == 0 &&
            state_sensor[3] == 1 &&
            state_sensor[4] == 0) {
    error = ERR_LEFT;
  }else if (state_sensor[0] == 0 &&
            state_sensor[1] == 0 &&
            state_sensor[2] == 0 &&
            state_sensor[3] == 1 &&
            state_sensor[4] == 1) {
    error = ERR_VERY_LEFT;
  }else if (state_sensor[0] == 0 &&
            state_sensor[1] == 0 &&
            state_sensor[2] == 0 &&
            state_sensor[3] == 0 &&
            state_sensor[4] == 1) {
    error = ERR_EXT_LEFT;
  }
}

void pid_calc() {
  if (error == 0) {
    integral = 0;
  }

  proportional = error;
  integral += error;

  if (integral > 255) {
    integral = 255;
  } else if (integral < -255) {
    integral = -255;
  }
  derivative = error - last_error;

  PID = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
//  Serial.print("ERROR: ");
//  Serial.println(error);
  last_error = error;
}

void motor_control() {
//  Serial.print("PID: ");
//  Serial.println(PID);
  //vira direita
  if (PID >= 0) {
    speed_right = speed_A - PID;
    speed_left = speed_A;
  } else {
    //virar esquerda
    speed_right = speed_A;
    speed_left = speed_A + PID;
  }

//  Serial.print("SPEED_R: ");
//  Serial.println(speed_right);
//  Serial.print("SPEED_L: ");
//  Serial.println(speed_left);

  analogWrite(motor_r_speed, speed_right);
  analogWrite(motor_l_speed, speed_left);
}