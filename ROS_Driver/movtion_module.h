// switch parts
int switch_pwm_A = 0;
int switch_pwm_B = 0;
bool usePIDCompute = true;
float spd_rate_A = 1.0;
float spd_rate_B = 1.0;
bool heartbeatStopFlag = false;

void movtionPinInit(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  ledcAttach(PWMA, freq, ANALOG_WRITE_BITS);
  ledcAttach(PWMB, freq, ANALOG_WRITE_BITS);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchEmergencyStop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void switchPortCtrlA(float pwmInputA){
  int pwmIntA = round(pwmInputA * spd_rate_A);
  if(abs(pwmIntA) < 1e-6){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  if(pwmIntA > 0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, pwmIntA);
  }
  else{
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA,-pwmIntA);
  }
}


void switchPortCtrlB(float pwmInputB){
  int pwmIntB = round(pwmInputB * spd_rate_B);
  if(abs(pwmIntB) < 1e-6){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if(pwmIntB > 0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, pwmIntB);
  }
  else{
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB,-pwmIntB);
  }
}


void switchCtrl(int pwmIntA, int pwmIntB) {
    switch_pwm_A = pwmIntA;
    switch_pwm_B = pwmIntB;
    switchPortCtrlA(switch_pwm_A);
    switchPortCtrlB(switch_pwm_B);
}


void lightCtrl(int pwmIn) {
  switch_pwm_A = pwmIn;
  switchPortCtrlA(-abs(switch_pwm_A));
}


void setSpdRate(float inputL, float inputR) {
  inputL = abs(inputL);
  if (inputL > 1) {
    inputL = 1;
  }
  inputR = abs(inputR);
  if (inputR > 1) {
    inputR = 1;
  }
  spd_rate_A = inputL;
  spd_rate_B = inputR;
}


void getSpdRate() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = CMD_GET_SPD_RATE;

  jsonInfoHttp["L"] = spd_rate_A;
  jsonInfoHttp["R"] = spd_rate_B;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}



// movtion parts.
// A-left, B-right

ESP32Encoder encoderA;
ESP32Encoder encoderB;

static unsigned long lastTime = 0;
static unsigned long lastLeftSpdTime = 0;
static unsigned long lastRightSpdTime = 0;
int lastEncoderA = 0;
int lastEncoderB = 0;

double speedGetA;
double speedGetB;

double pulsesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;


void initEncoders() {
  encoderA.attachHalfQuad(AENCA, AENCB);
  encoderB.attachHalfQuad(BENCA, BENCB);
  encoderA.setCount(0);
  encoderB.setCount(0);
}

double getEncoders(encoderState& encLeft, encoderState& encRight) {
  unsigned long currentTime = micros();
  // I am assuming these don't take very long
  encLeft.encoder = encoderA.getCount();
  encRight.encoder = encoderB.getCount();
  double dt = (double)(currentTime - encLeft.lastEncoderTime) / 1000000;
  encLeft.lastEncoderTime = currentTime;
  encRight.lastEncoderTime = currentTime;
  return dt;
}

double getEncodersSim(double pwm_left, double pwm_right, encoderState& encLeft, encoderState& encRight) {
  unsigned long currentTime = micros();
  double sign_left = (pwm_left > 0.0f) - (pwm_left < 0.0f);
  double sign_right = (pwm_right > 0.0f) - (pwm_right < 0.0f);
  double speed_left = sign_left * (fabs(pwm_left) - int_ff)/k_ff;
  double speed_right = sign_right * (fabs(pwm_right) - int_ff)/k_ff;
  double dt = (double)(currentTime - encLeft.lastEncoderTime) / 1000000;
  encLeft.encoder = speed_left * dt / pulsesRate + encLeft.lastEncoder;
  encRight.encoder = speed_right * dt / pulsesRate + encRight.lastEncoder;
  encLeft.lastEncoderTime = currentTime;
  encRight.lastEncoderTime = currentTime;
  return dt;
}

void getSpeed(
  double dt,
  encoderState& encoder
) {
  encoder.speed = (pulsesRate * (encoder.encoder - encoder.lastEncoder)) / dt;
  encoder.en_odom = encoder.encoder;
  if (SET_MOTOR_DIR) {
    encoder.speed = -encoder.speed;
    encoder.en_odom = -encoder.en_odom;
  }
  encoder.odom_updates += 1;
  encoder.lastEncoder = encoder.encoder;
}



// --- PID Controller ---

// PID_v2 pidA(__kp, __ki, __kd, PID::Direct);
// PID_v2 pidB(__kp, __ki, __kd, PID::Direct);

double setpointA = 0;
double setpointB = 0;

int setpoint_interval = 200;
unsigned long setpoint_cmd_recv = millis();
unsigned long setpoint_last_time = millis();
float setpointA_buffer;
float setpointB_buffer;
float setpointA_last;
float setpointB_last;
float change_offset = 0.005;
bool new_setpoint_flag = false;

void motorCtrl(double pwm, uint8_t pin1, uint8_t pin2, uint8_t channel){
  int pwmInt= round(pwm);
  if(SET_MOTOR_DIR){
    if(pwmInt < 0){
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      ledcWrite(channel, abs(pwmInt));
    }
    else{
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      ledcWrite(channel, abs(pwmInt));
    }
  }else{
    if(pwmInt < 0){
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
      ledcWrite(channel, abs(pwmInt));
    }
    else{
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
      ledcWrite(channel, abs(pwmInt));
    }
  }
}

void setGoalSpeed(float inputLeft, float inputRight) {
  usePIDCompute = true;

  if(inputLeft < -2.0 || inputLeft > 2.0){
    return;
  }

  if(inputRight < -2.0 || inputRight > 2.0){
    return;
  }
  
  setpointA = inputLeft;
  setpointB = inputRight;
}

double pwm_slew_limiter(double previous_pwm, double target_pwm) {
  // apply slew rate limits
  double dpwm = target_pwm - previous_pwm;
  if (dpwm > MAX_SLEW_PWM) dpwm = MAX_SLEW_PWM;
  else if (dpwm < -MAX_SLEW_PWM) dpwm = -MAX_SLEW_PWM;

  double new_pwm = previous_pwm + dpwm;

  // further limit sudden changes in motor direction by
  // injecting a zero step
  if (new_pwm * previous_pwm < 0.0) new_pwm = 0.0;

  return new_pwm;
}

double pwm_clamp(double pwm) {
  // apply pwm absolute limits
  if (abs(pwm) < THRESHOLD_PWM) pwm = 0.0;
  else if (pwm > MAX_PWM) pwm = MAX_PWM;
  else if (pwm < -MAX_PWM) pwm = -MAX_PWM;

  return pwm;
}

double pwm_feedforward(double setpoint) {
  double sign_setpoint = (setpoint > 0.0f) - (setpoint < 0.0f);
  double pwm = k_ff * setpoint + int_ff * sign_setpoint;
  return pwm;
}

double controller(double setpoint, double currentSpeed, double dt, motorControllerState& motorState) {
  // slew and clamp limited feed-forward/pid controller.
  // note this controller updates last_pwm and integral.
  //control error
  double error = setpoint - currentSpeed;

  // feedforward
  double pwm_ff = pwm_feedforward(setpoint);

  // pre-limit control
  double pwm_pid = __kp * error + motorState.integral;
  double pwm_target = pwm_ff + pwm_pid;

  // clamp
  pwm_target = pwm_clamp(pwm_target);

  // slew limit
  double pwm = pwm_slew_limiter(motorState.lastPwm, pwm_target);

  // anti-windup
  bool limiter_active = (fabs(pwm - pwm_target) > 1e-3);

  // some GPT ideas that seem unnecessary
  //bool pushing_further_positive = (pwm_target > pwm) && (error > 0.0);
  //bool pushing_further_negative = (pwm_target < pwm) && (error < 0.0);
  //bool block_integration = limiter_active && 
  //    (pushing_further_positive || pushing_further_negative);

  if (!limiter_active) {
    motorState.integral += __ki * error * dt;
  }

  // integral term limits
  if (motorState.integral > MAX_PWM) motorState.integral = MAX_PWM;
  if (motorState.integral < -MAX_PWM) motorState.integral = -MAX_PWM;

  // update pwm memory
  motorState.lastPwm = pwm;

  return pwm;
}

void dual_controller(
  double setpointLeft,
  double setpointRight,
  double currentSpeedLeft,
  double currentSpeedRight,
  double dt, 
  motorControllerState& motorStateLeft,
  motorControllerState& motorStateRight,
  motorControllerState& motorStateAvg,
  motorControllerState& motorStateDiff
  ) {
  // slew and clamp limited feed-forward/pid controller assuming left and right motor drives
  // are coupled through their interaction with the driving surface (data indicates this).
  // This controller goes back and forth between L/R and vavg, vdiff spaces.
  //control error
  double setpointAvg = (setpointLeft + setpointRight) / 2.0;
  double setpointDiff = (setpointRight - setpointLeft) / 2.0;
  double currentSpeedAvg = (currentSpeedLeft + currentSpeedRight) / 2.0;
  double currentSpeedDiff = (currentSpeedRight - currentSpeedLeft) / 2.0;

  const double sp_eps = 1e-2;
  const double speed_eps = 1e-2;
  // kill unwanted integral windup
  if (fabs(setpointAvg) < sp_eps &&
      fabs(setpointDiff) < sp_eps &&
      fabs(currentSpeedAvg) < speed_eps &&
      fabs(currentSpeedDiff) < speed_eps) {
    motorStateAvg.integral = 0.0;
    motorStateDiff.integral = 0.0;
  }
  //kill integral windups on sign change
  if (motorStateDiff.setpoint * setpointDiff < 0.0)
  {
    motorStateDiff.integral = 0.0;
  }
  if (motorStateAvg.setpoint * setpointAvg < 0.0)
  {
    motorStateAvg.integral = 0.0;
  }

  motorStateDiff.setpoint = setpointDiff;
  motorStateAvg.setpoint = setpointAvg;


  double errorAvg = setpointAvg - currentSpeedAvg;
  double errorDiff = setpointDiff - currentSpeedDiff;

  // feedforward
  double pwm_ff_avg = pwm_feedforward(setpointAvg);
  double pwm_ff_diff = pwm_feedforward(setpointDiff);

  // pre-limit control
  double pwm_pid_avg = __kp * errorAvg + motorStateAvg.integral;
  double pwm_target_avg = pwm_ff_avg + pwm_pid_avg;
  double pwm_pid_diff = __kp * errorDiff + motorStateDiff.integral;
  double pwm_target_diff = pwm_ff_diff + pwm_pid_diff;

  // back to L/R for limiting
  double pwm_target_left = pwm_target_avg - pwm_target_diff;
  double pwm_target_right = pwm_target_avg + pwm_target_diff;

  // clamp
  pwm_target_left = pwm_clamp(pwm_target_left);
  pwm_target_right = pwm_clamp(pwm_target_right);

  // slew limit
  double pwm_left = pwm_slew_limiter(motorStateLeft.lastPwm, pwm_target_left);
  double pwm_right = pwm_slew_limiter(motorStateRight.lastPwm, pwm_target_right);

  double pwm_avg = (pwm_left + pwm_right) / 2.0;
  double pwm_diff = (pwm_right - pwm_left) / 2.0;

  // anti-windup
  bool limiter_active_avg = (fabs(pwm_avg - pwm_target_avg) > 1e-3);
  bool limiter_active_diff = (fabs(pwm_diff - pwm_target_diff) > 1e-3);

  if (!limiter_active_avg) {
    motorStateAvg.integral += __ki * errorAvg * dt;
  }

  if (!limiter_active_diff) {
    motorStateDiff.integral += __ki * errorDiff * dt;
  }

  // integral term limits
  if (motorStateAvg.integral > MAX_PWM) motorStateAvg.integral = MAX_PWM;
  if (motorStateAvg.integral < -MAX_PWM) motorStateAvg.integral = -MAX_PWM;
  if (motorStateDiff.integral > MAX_PWM) motorStateDiff.integral = MAX_PWM;
  if (motorStateDiff.integral < -MAX_PWM) motorStateDiff.integral = -MAX_PWM;

  // update pwm memory
  // first two are settings for the motors
  motorStateLeft.pwm = pwm_left;
  motorStateRight.pwm = pwm_right;
  // yes, this is a little silly
  motorStateLeft.lastPwm = pwm_left;
  motorStateRight.lastPwm = pwm_right;
  // include avg and diff just for completeness
  motorStateAvg.pwm = pwm_avg;
  motorStateDiff.pwm = pwm_diff;
  // yes, this is a little silly
  motorStateAvg.lastPwm = pwm_avg;
  motorStateDiff.lastPwm = pwm_diff;
}


void setPID(float inputP, float inputI, float inputD, float inputLimits) {
  __kp = inputP;
  __ki = inputI;
  __kd = inputD;
  windup_limits = inputLimits;
  // pidA.SetTunings(__kp, __ki, __kd);
  // pidB.SetTunings(__kp, __ki, __kd);
}

void setFF(float inputK, float inputI) {
  k_ff = inputK;
  int_ff = inputI;
}

void rosCtrl(float rosX, float rosZ) {
  setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);
  setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);
  setGoalSpeed(setpointA, setpointB);
}

void heartBeatCtrl() {
  if (currentTimeMillis - lastCmdRecvTime > HEART_BEAT_DELAY) {
    if (!heartbeatStopFlag) {
      heartbeatStopFlag = true;
      setGoalSpeed(0, 0);
    }
  }
}

void changeHeartBeatDelay(int inputCmd) {
  HEART_BEAT_DELAY = inputCmd;
}

void mm_settings(byte inputMain, byte inputModule) {
  mainType = inputMain;
  moduleType = inputModule;
  
  // mainType:01 RaspRover
  // #define WHEEL_D 0.0800
  // #define ONE_CIRCLE_PLUSES  2100
  // #define TRACK_WIDTH  0.125
  // #define SET_MOTOR_DIR false

  // mainType:02 UGV Rover
  // #define WHEEL_D 0.0800
  // #define ONE_CIRCLE_PLUSES  1650(v=0.90) -> 660(v>=0.93)
  // #define TRACK_WIDTH  0.172
  // #define SET_MOTOR_DIR false

  // mainType:03 UGV Beast
  // #define WHEEL_D  0.0523
  // #define ONE_CIRCLE_PLUSES  1092
  // #define TRACK_WIDTH  0.141
  // #define SET_MOTOR_DIR true

  if (mainType == 1) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 2100;
    TRACK_WIDTH = 0.125;
    SET_MOTOR_DIR = false;
  } else if (mainType == 2) {
    WHEEL_D = 0.0800;
    ONE_CIRCLE_PLUSES = 660;
    TRACK_WIDTH = 0.172;
    SET_MOTOR_DIR = false;
  } else if (mainType == 3) {
    WHEEL_D = 0.0523;
    ONE_CIRCLE_PLUSES = 1092;
    TRACK_WIDTH = 0.141;
    SET_MOTOR_DIR = true;
  }
  pulsesRate = 3.14159265359 * WHEEL_D / ONE_CIRCLE_PLUSES;

  if (mainType == 1) {
    screenLine_2 = "RaspRover";
  } else if (mainType == 2) {
    screenLine_2 = "UGV Rover";
  } else if (mainType == 3) {
    screenLine_2 = "UGV Beast";
  } 

  if (moduleType == 0) {
    screenLine_2 += " Null";
  } else if (moduleType == 1) {
    screenLine_2 += " Arm";
  } else if (moduleType == 2) {
    screenLine_2 += " PT";
  } 
}