// <<<<<<<<<<=== === ===SSD1306: 0x3C=== === ===>>>>>>>>>>
// 0.91inch OLED
bool screenDefaultMode = true;

unsigned long currentTimeMillis = millis();
unsigned long lastTimeMillis = millis();

// default
String screenLine_0;
String screenLine_1;
String screenLine_2;
String screenLine_3;

// custom
String customLine_0;
String customLine_1;
String customLine_2;
String customLine_3;

// #include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// init oled ctrl functions.
void init_oled(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
}


// Updata all data and flash the screen.
void oled_update() {
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(screenLine_0);
  display.println(screenLine_1);
  display.println(screenLine_2);
  display.println(screenLine_3);

  display.display();
}

// dev info update on oled.
void oledInfoUpdate(encoderState encoderStateLeft,
	encoderState encoderStateRight,
	const motorControllerState motorStateLeft,
	const motorControllerState motorStateRight) {
  currentTimeMillis = millis();
  updates += 1;
  if (abs(motorStateLeft.lastPwm) > max_command_left){
    max_command_left = abs(motorStateLeft.lastPwm);
  }
  if (abs(motorStateRight.lastPwm) > max_command_right){
    max_command_right = abs(motorStateRight.lastPwm);
  }

  unsigned long dt_ms = currentTimeMillis - lastTimeMillis;
  unsigned long update_count = updates;
  float dt_s = dt_ms / 1000.0;
  double reported_max_command_left;
  double reported_max_command_right;
  if (dt_s > 5.0) {
    inaDataUpdate();
    a_rate = a_updates / dt_s;
    a_updates = 0;
    g_rate = g_updates / dt_s;
    g_updates = 0;
    m_rate = m_updates / dt_s;
    m_updates = 0;
    odom_l_rate = encoderStateLeft.odom_updates / dt_s;
    encoderStateLeft.odom_updates = 0;
    odom_r_rate = encoderStateRight.odom_updates / dt_s;
    encoderStateRight.odom_updates = 0;
    update_rate = updates / dt_s;
    updates = 0;
    reported_max_command_left = max_command_left;
    reported_max_command_right = max_command_right;
    max_command_left = 0;
    max_command_right = 0;
    lastTimeMillis = currentTimeMillis;
  } else {
    return;
  }
  if (!screenDefaultMode) {
    return;
  }
  //screenLine_0 = "l:" + String(odom_l_rate) + " r:" + String(odom_r_rate);
  //screenLine_1 = "a:" + String(a_rate) + " g:" + String(g_rate);
  // data/update rate and display rate
  screenLine_0 = "dr:" + String(g_rate) + " dt:" + String(dt_s);
  // max left and right PWM commands over the last interval
  screenLine_1 = "lp:" + String(reported_max_command_left) + " rp:" + String(reported_max_command_right);
  //screenLine_2 = "m:" + String(m_rate) + "dt:" + String(dt_s);
  //screenLine_2 = "kp:" + String(int(round(__kp))) + "kd:" + String(int(round(__kd))) + "ki:" + String(int(round(__ki)));
  // left and right encoder
  screenLine_2 = "le:" + String(encoderStateLeft.en_odom) + " re:" + String(encoderStateRight.en_odom);
  //screenLine_2 = "w:" + String(WHEEL_D) + " o:" + String(ONE_CIRCLE_PLUSES);
  //screenLine_3 = "uc:" + String(update_count) + "dt:" + String(dt_s);
  screenLine_3= "VL:" + String(loadVoltage_V) + " s " + String(mainType) + String(moduleType);
  //screenLine_3= "VL:" + String(loadVoltage_V) + " s " + String(mainType) + String(moduleType);
  oled_update();
}

// oled ctrl.
void oledCtrl(byte inputLineNum, String inputMegs) {
  screenDefaultMode = false;
  switch (inputLineNum) {
  case 0: customLine_0 = inputMegs;break;
  case 1: customLine_1 = inputMegs;break;
  case 2: customLine_2 = inputMegs;break;
  case 3: customLine_3 = inputMegs;break;
  }
  display.clearDisplay();
  display.setCursor(0,0);

  display.println(customLine_0);
  display.println(customLine_1);
  display.println(customLine_2);
  display.println(customLine_3);

  display.display();
}

// set oled as default.
void setOledDefault(){
  screenDefaultMode = true;
  inaDataUpdate();
  screenLine_3 = "V:"+String(loadVoltage_V);
  oled_update();
  lastTimeMillis = currentTimeMillis;
}