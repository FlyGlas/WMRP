// some preprocessor defines
#define SW_VERSION      "Version 1.2d"
#define DEBUG           true
//DIGITAL INPUT PINS
// Values after the // are the one for my pcb. Other for debugging on Arduino Uno!
#define PIN_SW_1         12
#define PIN_SW_2         5
#define PIN_SW_3         9
#define PIN_ROT_A        8
#define PIN_ROT_B        6
#define PIN_ROT_SW       4

#define DEBUG_LED        11 // STATUS_LED

//ADC INPUT PINS
#define PIN_ADC_T_GRIP   A1
#define PIN_ADC_T_TIP    A0
#define PIN_ADC_U_IN     A4
#define PIN_ADC_I_HEATER A3
#define PIN_ADC_STAND    A2

//OUTPUT PINS
#define PIN_HEATER       10
#define PIN_STAND        13
//#define PIN_I2C_SDA    2
//#define PIN_I2C_SCL    3
//#define PIN_MOSI       16
//#define PIN_MISO       17
//#define PIN_SCK        15
//#define PIN_RXI        0
//#define PIN_TXO        1

//TIMES
#define TIMER1_PERIOD_US        50
#define DELAY_BEFORE_MEASURE_MS 5
#define TIME_SW_POLL_MS         10
#define TIME_TUI_MEAS_MS        100
#define TIME_SERIAL_MS          500
#define TIME_LCD_MS             500
#define TIME_CYCLECOUNT_MS      1000
#define TIME_EEPROM_WRITE_MS    30000
#define TIME_STAND_MS           500
#define DELAY_BEFORE_STAND_MS   5
#define TIME_ERROR_MS           50
#define TIME_BLINK_LCD          150
#define TIME_SLEEP_S            360 //60*6
#define TIME_VERSION            2000

//THRESHOLDS
#define THRESHOLD_STAND         300      //adc value
#define ERROR_COUNTER_THRESHOLD 30       //* TIME_ERROR_MS (50) = 1500ms
#define VOLTAGE_TRESHOLD        10500000 //10.5V

//PID CONTROL - SOLDERING
#define CNTRL_P_GAIN            40.0
#define CNTRL_I_GAIN            8.0
#define CNTRL_D_GAIN            0.0

//PID CONTROL - TARGET TEMP && PWM < 10%
#define BAND_TARGET_TEMP_GRAD   4
#define BAND_MAX_PWM_PERCENT    10.0
#define BAND_P_GAIN             10.0
#define BAND_I_GAIN             2.0
#define BAND_D_GAIN             0.0

//TEMPERATURES
#define STDBY_TEMP_DEG         150
#define MIN_TARGET_TEMP_DEG    150
#define MAX_TARGET_TEMP_DEG    450

//PWM MAX
#define PWM_MAX_VALUE          1023

//ADC TO TIP TEMPERATURE CONVERSION
#define ADC_TO_T_TIP_A0  -1176200 //                  T[degC] = -1176.2 + 3.34 * SQRT(14978 * U[mV] + 130981)
#define ADC_TO_T_TIP_A1  3340     //                    U[mV] = adc_value * 4mV * 1/OPAMP_T_TIP_GAIN
#define ADC_TO_T_TIP_A2  139      //           10^3 * T[degC] = -1176200 + 3340 * SQRT(139 * adc_value + 130981)
#define ADC_TO_T_TIP_A3  130981

//ADC TO GRIP TEMPERATURE CONVERSION
#define ADC_TO_T_GRIP_A0 289      //                  T[degC] = (U[V] - 1.1073) * 1000/6.91
#define ADC_TO_T_GRIP_A1 -160246  //                     U[V] = adc_value * 4 V/1000 * 1/OPAMP_T_GRIP_GAIN
//                                             10^3 * T[degC] = adc_value * 289.435 - 160246

//ADC TO INPUT VOLTAGE CONVERSION
#define ADC_TO_U_IN_A0 14213      //                 U_adc[V] = U_in[V] * 4700/(4700 + 12000);
//                                                   U_adc[V] = adc_value * 4 V/1000
//                                                    U_in[V] = adc_value * 4 V/1000 *(4700 + 12000)/4700
//                                             10^6 * U_in[V] = adc_value * 14212,77

//ADC TO INPUT CURRENT CONVERSION
#define ADC_TO_I_HEATER_A0 18182  //                U_adc[mV] = UVCC/2[mV] + I_heater[A] * 220 mV/A
//                                                  U_adc[mV] = adc_value * 4 mV
//                                                 UVCC/2[mV] = adc_value(@i=0A) * 4 mV
//                                                I_heater[A] = (adc_value * 4 mV - UVCC/2[mV])/(220 mV/A)
//                                         10^6 * I_heater[A] = (adc_value - adc_value(@i=0A)) * 18182

//#define OPAMP_T_TIP_GAIN    431
//#define OPAMP_T_GRIP_GAIN   2
//#define OPAMP_I_HEATER_GAIN 2.2

//EEPROM ADDRESS
#define EEPROM_ADDRESS_TEMP_START   0
//#define EEPROM_ADDRESS_TEMP_END   1
#define EEPROM_ADDRESS_VAL1_START   2
//#define EEPROM_ADDRESS_VAL1_END   3
#define EEPROM_ADDRESS_VAL2_START   4
//#define EEPROM_ADDRESS_VAL2_END   5
#define EEPROM_ADDRESS_VAL3_START   6
//#define EEPROM_ADDRESS_VAL3_END   7


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "TimerOne.h"          //Version r11   --> http://code.google.com/p/arduino-timerone/
#include <EEPROM.h>            //Included in Arduino IDE Version 1.6.0
#include <Wire.h>              //Included in Arduino IDE Version 1.6.0
#include <LiquidCrystal_I2C.h> //Version 1.2.1 --> https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#include <Encoder.h>           //Version 1.2   --> http://www.pjrc.com/teensy/td_libs_Encoder.html
Encoder enc(PIN_ROT_A, PIN_ROT_B);

//GLOBAL VARIABLES
long    temperature_tip_absolute;
int adc_temperature_tip_relative;
int adc_temperature_tip_relative_old;
long    temperature_tip_relative;
int adc_temperature_grip;
long    temperature_grip;
int adc_voltage_input;
long    voltage_input;
int adc_current_heater;
long    current_heater;

int adc_current_heater_offset;
int pwm_value;
int pwm_value_mean;
byte pwm_percent;
int temp_setpoint;
int temp_setpoint_old;
int encoder_value;

boolean state_switch[5];
boolean state_switch_old[5];// = {false, true, true, true};
byte button_count[5];

boolean meas_flag;

boolean sleep_flag;
long    sleep_counter = TIME_SLEEP_S;
boolean status_stand_reed;
boolean status_stand_manu = true;  //start with standby

boolean error_voltage;
boolean temp_flag;
boolean error_tip;
boolean no_tip;
byte error_counter;

byte pid_flag;

//variables for cycle count
int cycle;
int last_cycle;

//chars for bargraph
byte p1[8] = {
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000
};

byte p2[8] = {
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000
};

byte p3[8] = {
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100
};

byte p4[8] = {
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110
};

byte box_char[8] = {
  0b00000,
  0b00000,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b00000,
  0b00000
};
byte temperatur_char[8] = {
  0b10010,
  0b11010,
  0b10010,
  0b11010,
  0b10010,
  0b11110,
  0b11110,
  0b11110
};
byte gradcelsius_char[8] = {
  0b01000,
  0b10100,
  0b01000,
  0b00011,
  0B00100,
  0b00100,
  0b00100,
  0b00011
};
byte arrow_char[8] = {
  0b11111,
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};

const byte eeprom_address[3] = {
  EEPROM_ADDRESS_VAL1_START,
  EEPROM_ADDRESS_VAL2_START,
  EEPROM_ADDRESS_VAL3_START
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CLASSES

//timer_over class
class timer_over {
  private:
    unsigned long _last_time;
  public:
    timer_over() {
      _last_time = 0;
    }
    boolean over(long sample_time) {
      unsigned long now = millis();
      unsigned long time_change = (now - this->_last_time);

      if (time_change >= sample_time) {
        this->_last_time = now;
        return true;
      }
      else {
        return false;
      }
    }
    boolean set_timer() {
      this->_last_time = millis();
    }
};
//timer_over class members
timer_over timer_meas;
timer_over timer_serial;
timer_over timer_lcd;
timer_over timer_switch;
timer_over timer_cyclecount;
timer_over timer_eeprom;
timer_over timer_delay_before_measure;
timer_over timer_stand;
timer_over timer_error_chk;
timer_over timer_blink_lcd;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS

//Original code for bargraph --> http://playground.arduino.cc/Code/LcdBarGraph
void draw_bar(int value, byte numCols, int maxValue) {
  static int prevValue;
  static byte lastFullChars;

  byte fullChars = (long)value * numCols / maxValue;
  byte mod = ((long)value * numCols * 5 / maxValue) % 5;

  int normalizedValue = (int)fullChars * 5 + mod;

  if (prevValue != normalizedValue) {

    for (byte i = 0; i < fullChars; i++) {
      lcd.write((char)0b11111111);
    }
    if (mod > 0) {
      switch (mod) {
        case 0:
          break;
        case 1:
          lcd.print((char)0);
          break;
        case 2:
          lcd.write(1);
          break;
        case 3:
          lcd.write(2);
          break;
        case 4:
          lcd.write(3);
          break;
          ++fullChars;
      }
    }
    //for (byte i = fullChars; i < lastFullChars; i++) {
    for (byte i = fullChars; i < numCols; i++) {
      lcd.write(' ');
    }

    // -- save cache
    lastFullChars = fullChars;
    prevValue = normalizedValue;
  }
}


void update_measurments() {
  //empty
}


int calculate_pid(int setpoint, int input, float kp, float ki, float kd, int sample_time, int out_max) {
  //float pid(float setpoint, float input, float kp, float ki, float kd, unsigned int sample_time, float out_max, float &i_value, float &p_value) {
  static unsigned long last_time;
  static float last_input;
  static float iterm;
  float error;
  float dterm;
  float pterm;
  float output;
  float out_min = 0;

  float sample_time_in_sec = ((float)sample_time) / 1000;

  kp = kp;
  ki = ki * sample_time_in_sec;
  kd = kd / sample_time_in_sec;

  // compute all the working error variables
  error = setpoint - input;

  // (d)erivative
  dterm = kd * (input - last_input);

  // (p)roportional
  pterm = kp * error;

  // (i)ntegral with anti wind up
  if ((dterm + pterm + iterm) <= out_max) {
    iterm += (ki * error);
  }

  if (iterm > out_max) iterm = out_max;
  else if (iterm < out_min) iterm = out_min;

  // compute PID output
  output = pterm + iterm - dterm;

  if (output > out_max) output = out_max;
  else if (output < out_min) output = out_min;

  // Remember some variables for next time
  last_input  = input;
  return (int)output;
}


long adc_to_t_tip_calc(int adc_value) {
  long t_tip = ADC_TO_T_TIP_A0 + (ADC_TO_T_TIP_A1 * sqrt((ADC_TO_T_TIP_A2 * (long)adc_value) + ADC_TO_T_TIP_A3));
  //Serial.println((float)t_tip / 1000, 3);
  return t_tip;
}


long adc_to_t_grip_calc(int adc_value) {
  long t_grip = (ADC_TO_T_GRIP_A0 * (long)adc_value) + ADC_TO_T_GRIP_A1;
  //Serial.println((float)t_grip / 1000, 3);
  return t_grip;
}


long adc_to_u_in_calc(int adc_value) {
  long u_in = (ADC_TO_U_IN_A0 * (long)adc_value);
  //Serial.println((float)u_in / 1000000, 3);
  return u_in;
}


long adc_to_i_heater_calc(int adc_value, int adc_value_offset) {
  long i_heater = ADC_TO_I_HEATER_A0 * ((long)adc_value - (long)adc_value_offset);
  //Serial.println((float)i_heater / 1000000, 3);
  return i_heater;
}


void  serial_draw_line() {
  Serial.println("--------------------------------------");
}

int eeprom_read_int(int addr) {
  int value = EEPROM.read(addr) | (((int) EEPROM.read(addr + 1)) << 8);
  value = constrain(value, MIN_TARGET_TEMP_DEG, MAX_TARGET_TEMP_DEG);
  if (DEBUG) {
    Serial.print("Read from EEPROM Address: ");
    Serial.print(addr);
    Serial.print("/");
    Serial.println(addr + 1);
    Serial.print("Value: ");
    Serial.println(value);
    serial_draw_line();
  }
  return value;
}

void eeprom_write_int(int addr, int value) {
  EEPROM.write(addr, value & 0xff);
  EEPROM.write(addr + 1, (value & 0xff00) >> 8);

  if (DEBUG) {
    Serial.print("Write to EEPROM Address: ");
    Serial.print(addr);
    Serial.print("/");
    Serial.println(addr + 1);
    Serial.print("Value: ");
    Serial.println(value);
    serial_draw_line();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP ROUTINE

void setup()
{
  //USE EXTERNAL AREF
  analogReference(EXTERNAL);

  //OUTPUT PINS
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_STAND, OUTPUT);

  pinMode(DEBUG_LED, OUTPUT);


  //DIGITAL INPUT PINS
  pinMode(PIN_SW_1, INPUT);
  digitalWrite(PIN_SW_1, HIGH);
  pinMode(PIN_SW_2, INPUT);
  digitalWrite(PIN_SW_2, HIGH);
  pinMode(PIN_SW_3, INPUT);
  digitalWrite(PIN_SW_3, HIGH);
  pinMode(PIN_ROT_A, INPUT);
  digitalWrite(PIN_ROT_A, HIGH);
  pinMode(PIN_ROT_B, INPUT);
  digitalWrite(PIN_ROT_B, HIGH);
  pinMode(PIN_ROT_SW, INPUT);
  digitalWrite(PIN_ROT_SW, HIGH);

  //ADC INPUT PINS
  pinMode(PIN_ADC_T_GRIP, INPUT);
  pinMode(PIN_ADC_U_IN, INPUT);
  pinMode(PIN_ADC_I_HEATER, INPUT);
  pinMode(PIN_ADC_STAND, INPUT);


  Timer1.initialize(TIMER1_PERIOD_US); //initialize timer1, and set a 50 second period == 20 kHz
  Timer1.attachInterrupt(isr_routine); //attaches callback() as a timer overflow interrupt
  Timer1.pwm(PIN_HEATER, 0);           //setup pwm pin, 0 % duty cycle

  delay(100);
  adc_current_heater_offset = analogRead(PIN_ADC_I_HEATER); //read adc offset for current measurement @ I_HEATER = 0 A
  adc_temperature_grip      = analogRead(PIN_ADC_T_GRIP);   //read grip temperatur to avoid error at startup

  temp_setpoint = eeprom_read_int(EEPROM_ADDRESS_TEMP_START); //read eeprom
  temp_setpoint_old = temp_setpoint;

  enc.write(temp_setpoint);   //set encoder value to temp_setpoint

  Serial.begin(115200);       //start serial communication
  Serial.println(SW_VERSION); //print software version

  delay(10);
  lcd.begin(16, 2);      //initialize the lcd for 16 chars 2 lines
  lcd.createChar(0, p1); //create chars for bargraph
  lcd.createChar(1, p2);
  lcd.createChar(2, p3);
  lcd.createChar(3, p4);
  lcd.createChar(4, box_char);
  lcd.createChar(5, temperatur_char);
  lcd.createChar(6, gradcelsius_char);
  lcd.createChar(7, arrow_char);
  lcd.clear();
  delay(10);

  //start screen on lcd
  lcd.setCursor(2, 0); //Start at character 0 on line 0
  lcd.write("WMRP STATION");
  lcd.setCursor(2, 1); //Start at character 0 on line 0
  lcd.write(SW_VERSION);
  delay(TIME_VERSION);
  lcd.clear();
}

void isr_routine()
{
  //isr routine - not used now
}

void loop()
{
  //digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_cyclecount.over(TIME_CYCLECOUNT_MS)) {
    //here comes the cycles per second counter
    last_cycle = cycle;
    cycle = 0;

    if (status_stand_reed || status_stand_manu) {
      if (sleep_counter > 0) {
        sleep_counter--;
      }
    }
    else {
      sleep_counter = TIME_SLEEP_S;
    }
    if (sleep_counter == 0) {
      sleep_flag = true;
    }
    else {
      sleep_flag = false;
    }

  }
  else {
    cycle++;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_eeprom.over(TIME_EEPROM_WRITE_MS)) {
    //here comes the eeprom write
    if (temp_setpoint != temp_setpoint_old) {
      eeprom_write_int(EEPROM_ADDRESS_TEMP_START, temp_setpoint);
      temp_setpoint_old = temp_setpoint;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_meas.over(TIME_TUI_MEAS_MS)) {
    //here comes the start of the measurement
    adc_current_heater = analogRead(PIN_ADC_I_HEATER);
    adc_voltage_input  = analogRead(PIN_ADC_U_IN);

    Timer1.setPwmDuty(PIN_HEATER, 0);         //stop heating
    meas_flag = true;                         //set meas_flag
    timer_delay_before_measure.set_timer();   //set delay timer to actual time
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_delay_before_measure.over(DELAY_BEFORE_MEASURE_MS) && meas_flag) {
    meas_flag = false;
    //delay(DELAY_BEFORE_MEASURE_MS);   //wait for steady state of all filters and opmap (the hard way with delays)
    adc_temperature_grip         = analogRead(PIN_ADC_T_GRIP);
    adc_temperature_tip_relative_old = adc_temperature_tip_relative;
    adc_temperature_tip_relative = (analogRead(PIN_ADC_T_TIP)
                                    + analogRead(PIN_ADC_T_TIP)
                                    + analogRead(PIN_ADC_T_TIP)
                                    + analogRead(PIN_ADC_T_TIP)
                                    + analogRead(PIN_ADC_T_TIP)) / 5;

    //code for stand recognition....
    if (timer_stand.over(TIME_STAND_MS)) {
      digitalWrite(PIN_STAND, 1);
      delay(DELAY_BEFORE_STAND_MS);
      if (analogRead(PIN_ADC_STAND) < THRESHOLD_STAND) {
        status_stand_reed = true;
        status_stand_manu = false;
      }
      else {
        status_stand_reed = false;
      }
      digitalWrite(PIN_STAND, 0);
    }

    Timer1.setPwmDuty(PIN_HEATER, pwm_value); //set pwm back to old value

    //calculate SI values from the adc value
    temperature_tip_relative = adc_to_t_tip_calc(adc_temperature_tip_relative);                     //temperature in grad celsius divide with 1000
    temperature_grip         = adc_to_t_grip_calc(adc_temperature_grip);                            //temperature in grad celsius divide with 1000
    voltage_input            = adc_to_u_in_calc(adc_voltage_input);                                 //voltage in volt divide with 10^6
    current_heater           = adc_to_i_heater_calc(adc_current_heater, adc_current_heater_offset); //current in volt divide with 10^6

    //calculate real tip temperature as sum of thermo couple temperature and cold junction temperature
    temperature_tip_absolute = temperature_tip_relative + temperature_grip;

    //temp_setpoint = constrain(temp_setpoint, MIN_TARGET_TEMP_DEG, MAX_TARGET_TEMP_DEG);
    if (status_stand_reed || status_stand_manu) {
      pwm_value = calculate_pid(STDBY_TEMP_DEG, (int)(temperature_tip_absolute / 1000), BAND_P_GAIN, BAND_I_GAIN, BAND_D_GAIN, TIME_TUI_MEAS_MS, PWM_MAX_VALUE);
      pid_flag = 0;
    }
    else {
      if (abs(temp_setpoint - (int)(temperature_tip_absolute / 1000)) < BAND_TARGET_TEMP_GRAD && pwm_value < BAND_MAX_PWM_PERCENT * PWM_MAX_VALUE / 100) {
        pwm_value = calculate_pid(temp_setpoint, (int)(temperature_tip_absolute / 1000), BAND_P_GAIN, BAND_I_GAIN, BAND_D_GAIN, TIME_TUI_MEAS_MS, PWM_MAX_VALUE);
        pid_flag = 0;
      }
      else {
        pwm_value = calculate_pid(temp_setpoint, (int)(temperature_tip_absolute / 1000), CNTRL_P_GAIN, CNTRL_I_GAIN, CNTRL_D_GAIN, TIME_TUI_MEAS_MS, PWM_MAX_VALUE);
        pid_flag = 1;
      }
    }
    //pwm_value = 60;
    //no heating when error event occoured
    if (error_tip || no_tip || error_voltage || sleep_flag) {
      pwm_value = 0;
    }

    Timer1.setPwmDuty(PIN_HEATER, pwm_value);  //set pwm to new value


    //pwm value for lcd
    if (pwm_value < PWM_MAX_VALUE / 3) {
      pwm_value_mean = (8 * pwm_value_mean  + 2 * pwm_value) / 10;
    }
    else {
      pwm_value_mean = (0 * pwm_value_mean + 10 * pwm_value) / 10;
    }
    //Serial.println("Timer over...");
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_serial.over(TIME_SERIAL_MS)) {
    //here comes the serial output
    if (0) {
      //Serial data for csv export
      Serial.print((float)temperature_tip_absolute / 1000, 3);
      Serial.print(";");
      Serial.print(temp_setpoint);
      Serial.print(";");
      Serial.print(pwm_value);
      Serial.print(";");
      Serial.println((float)current_heater / 1000000, 3);
      //Serial.print(";");
      //erial.println(analogRead(PIN_ADC_STAND));
    }
    if (1) {
      //serial data for visualization in serial monitor
      Serial.print("Input Voltage:       ");
      Serial.println((float)voltage_input / 1000000, 3);
      Serial.print("Heater Current:      ");
      Serial.println((float)current_heater / 1000000, 3);
      Serial.print("Temp Grip:           ");
      Serial.println((float)temperature_grip / 1000, 3);
      Serial.print("Temp Tip (relative): ");
      Serial.println((float)temperature_tip_relative / 1000, 3);
      Serial.print("Temp Tip (absolute): ");
      Serial.println((float)temperature_tip_absolute / 1000, 3);
      Serial.print("Setpoint Temp:       ");
      Serial.println(temp_setpoint);
      Serial.print("PWM:                 ");
      Serial.println(pwm_value);
      Serial.print("PWM (mean):          ");
      Serial.println(pwm_value_mean);
      Serial.print("Status Stand (Reed): ");
      Serial.println(status_stand_reed);
      Serial.print("Status Stand (Manu): ");
      Serial.println(status_stand_manu);
      Serial.print("Cycles per sec:      ");
      Serial.println(last_cycle);
      Serial.print("Error Tip:           ");
      Serial.println(error_tip);
      Serial.print("No Tip:              ");
      Serial.println(no_tip);
      Serial.print("Temp flag:           ");
      Serial.println(temp_flag);
      Serial.print("Error counter:       ");
      Serial.println(error_counter);
      Serial.print("Error voltage:       ");
      Serial.println(error_voltage);
      Serial.print("Sleep counter:       ");
      Serial.println(sleep_counter);
      Serial.print("Sleep flag:          ");
      Serial.println(sleep_flag);

      serial_draw_line();
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_lcd.over(TIME_LCD_MS)) {
    //here comes the lcd output
    //first line
    //"temperature symbol (0), temp_tip_abs (1,2,3), grad celsius symbol (4), blank (5), setpoint symbol (6), temp_set (7,8,9), grad celsius symbol (10)"
    lcd.setCursor(0, 0); //Start at character 0 on line 0
    lcd.write(byte(5));
    if ((int)(temperature_tip_absolute / 1000) < (MIN_TARGET_TEMP_DEG - 50) || (int)(temperature_tip_absolute / 1000) > (MAX_TARGET_TEMP_DEG + 50) || error_tip || no_tip) {
      lcd.print(" - ");
    }
    else {
      if (int(temperature_tip_absolute / 1000) < 10)   lcd.print(" ");
      if (int(temperature_tip_absolute / 1000) < 100)  lcd.print(" ");
      lcd.print(int(temperature_tip_absolute / 1000));
    }
    lcd.write(byte(6));

    lcd.setCursor(5, 0); //Start at character 6 on line 0
    lcd.print(" ");

    lcd.setCursor(6, 0); //Start at character 6 on line 0
    lcd.write(byte(7));
    if (status_stand_reed || status_stand_manu) {
      lcd.print(STDBY_TEMP_DEG);
    }
    else {
      lcd.print(temp_setpoint);
    }
    lcd.write(byte(6));

    lcd.setCursor(12, 0); //Start at character 6 on line 0
    if (!error_tip && !no_tip && !error_voltage) {
      if (status_stand_reed || status_stand_manu) {
        if (sleep_flag) {
          lcd.print("OFF ");
        }
        else {
          if (sleep_counter > 60) {
            lcd.print("STBY");
          }
          else {
            lcd.print(" ");
            if (sleep_counter < 10) lcd.print(" ");
            lcd.print(sleep_counter);
            lcd.print(" ");
          }
        }
      }
      else {
        lcd.print("HEAT");
      }
    }
    else {
      if (error_voltage) {
        lcd.print("BATT");
      }
      else {
        lcd.print("ERR ");
      }
    }


    //second line
    //pid_flag
    lcd.setCursor(0, 1);
    if (pid_flag) {
      lcd.write((char)4); //heating or not in band around the setpoint
    }
    else {
      lcd.print((char)0b11011011); //in band around the setpoint (Chars: http://de.wikipedia.org/wiki/HD44780#/media/File:Charset.gif)
    }

    //pwm in percent (0,1,2), % (3), blank (4), bargraph (5-15)
    pwm_percent = map(pwm_value_mean, 0, PWM_MAX_VALUE, 0, 100);
    lcd.setCursor(1, 1); //Start at character 0 on line 1
    if (pwm_percent < 10)  lcd.print(" ");
    if (pwm_percent < 100) lcd.print(" ");
    lcd.print(pwm_percent);
    lcd.print("%");

    lcd.setCursor(6, 1); //Start at character 5 on line 1
    draw_bar(pwm_percent, 10, 100);

    //toogle led if in temperature range...
    if (abs(temp_setpoint - (int)(temperature_tip_absolute / 1000)) < BAND_TARGET_TEMP_GRAD) {
      digitalWrite(DEBUG_LED, true);
    }
    else {
      digitalWrite(DEBUG_LED, false);
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_switch.over(TIME_SW_POLL_MS)) {
    //here comes the switch reading
    state_switch[0] = !digitalRead(PIN_SW_1);
    state_switch[1] = !digitalRead(PIN_SW_2);
    state_switch[2] = !digitalRead(PIN_SW_3);
    state_switch[3] = !digitalRead(PIN_ROT_SW);

    //the three buttons under the lcd
    for (int i = 0; i < 3; i++) {
      if (state_switch[i] && state_switch_old[i]) {
        if (button_count[i] == 100) {
          eeprom_write_int(eeprom_address[i], temp_setpoint);  //safe temperature value to eeprom if buttonCount is equal 100
          timer_blink_lcd.set_timer();
          lcd.noBacklight();
        }
        button_count[i]++;                                     //count up if button is and was pushed
      }
      else {
        if (!state_switch[i] && state_switch_old[i]) {         //detect falling edge, releasing the button
          temp_setpoint = eeprom_read_int(eeprom_address[i]);  //read temp from eeeprom
          enc.write(temp_setpoint);
        }
        button_count[i] = 0;                                   //resett counter
      }
    }

    //the button included in the rotary encoder
    if (!state_switch[3] && state_switch_old[3]) {
      if (!error_tip && !no_tip) {
        status_stand_manu = !status_stand_manu;
      }
      else {
        error_tip = false;
        no_tip = false;
        temp_flag = false;
        error_counter = 0;
      }
    }

    memcpy(&state_switch_old, &state_switch, sizeof(state_switch)); // save values
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (timer_blink_lcd.over(TIME_BLINK_LCD)) {
    lcd.backlight();
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (timer_error_chk.over(TIME_ERROR_MS)) {

    if (adc_temperature_tip_relative > 100 && !temp_flag) { 	//set temp_flag = true after adc_temperature_tip_relative > 100
      temp_flag = true;
    }

    if (!sleep_flag) {
      if (adc_temperature_tip_relative < 30) {
        if (temp_flag == true) { 			         //adc_temperature_tip_relative < 30 but was > 100 before
          error_tip = true;
          Serial.println("ERROR: Temperature droped below 30 digits.");
        }
        else {						        //adc_temperature_tip_relative < 30 and never before > 100
          error_counter++;
          if (error_counter > ERROR_COUNTER_THRESHOLD) { 	//error after x cycles without adc_temperature_tip_relative > 100
            error_tip = true;
            //Serial.println("ERROR: Temperature rise at startup not fast enought.");
            error_counter = 0;
          }
        }
      }
      else {
        //error_tip = false;
      }
    }
    else {
      temp_flag = false;
    }

    if (adc_temperature_tip_relative > 900) {
      no_tip = true;
      //Serial.println("ERROR: Tip temperature greater than 900 digitis.");
      //error_tip = false;
    }
    else {
      no_tip = false;
    }

    if ((adc_temperature_tip_relative + 80) < adc_temperature_tip_relative_old) { //1K = 2 ADC digits
      error_tip = true;
      //Serial.println("ERROR: Last tip temperature 80 digits greater than actual temperatur.");
    }

    if (adc_temperature_grip < 500 || adc_temperature_grip > 755) { //t_grip < ~-10°C or t_grip > ~60°C
      error_tip = true;
      //Serial.println("ERROR: Grip temperature under -10C or above 60C.");
    }

    if (voltage_input < VOLTAGE_TRESHOLD && pwm_value > 0) {
      error_voltage = true;
      //Serial.println("ERROR: Battery undervoltage.");
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (!error_tip && !no_tip && !status_stand_manu && !status_stand_reed && !error_voltage) {
    //here comes the rotary encoder reading (every loop, when no error or in stand)
    encoder_value = enc.read();
    if (abs(temp_setpoint - encoder_value) >= 4) {
      temp_setpoint = temp_setpoint + (encoder_value - temp_setpoint) / 4; //read rotary encoder
      temp_setpoint = constrain(temp_setpoint, MIN_TARGET_TEMP_DEG, MAX_TARGET_TEMP_DEG);
      enc.write(temp_setpoint);
    }
  }
}

