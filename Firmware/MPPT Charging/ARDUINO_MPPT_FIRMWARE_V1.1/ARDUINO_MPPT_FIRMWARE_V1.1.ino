         
#include <EEPROM.h>                 
#include <Wire.h>                   
#include <SPI.h>                    
#include <WiFi.h>                   
#include <WiFiClient.h>             
#include <LiquidCrystal_I2C.h>   
#include <Adafruit_ADS1X15.h>       

#include <HTTPClient.h>

LiquidCrystal_I2C lcd(0x27,16,2);   
TaskHandle_t Core2;                 
//Adafruit_ADS1015 ads;              
Adafruit_ADS1115 ads;             

#define backflow_MOSFET 32          //SYSTEM PARAMETER - Backflow MOSFET
#define backflow_BAT    27          //SYSTEM PARAMETER - Backflow BAT
#define buck_IN         15          //SYSTEM PARAMETER - Buck MOSFET Driver PWM Pin
#define buck_EN         2          //SYSTEM PARAMETER - Buck MOSFET Driver Enable Pin
#define LED             2           //SYSTEM PARAMETER - LED Indicator GPIO Pin
#define FAN             13         //SYSTEM PARAMETER - Fan GPIO Pin
#define ADC_ALERT       34          //SYSTEM PARAMETER - Fan GPIO Pin
#define TempSensor      33         //SYSTEM PARAMETER - Temperature Sensor GPIO Pin
#define buttonLeft      17         //SYSTEM PARAMETER - 
#define buttonRight     16          //SYSTEM PARAMETER -
#define buttonBack      18          //SYSTEM PARAMETER - 
#define buttonSelect    5          //SYSTEM PARAMETER -

const char * ssid = "iPhone 11";
const char * password = "00000009";
String GOOGLE_SCRIPT_ID = "AKfycbwJOtu8S28yvct1m-rx_Nt6WHLx2As8JCBP0yWKZ7dLOXc_P4KpcntIv8DBzz1kX4VzZQ";

unsigned long Time;
String param = "";

bool                                  
MPPT_Mode               = 1,           //   USER PARAMETER - Enable MPPT algorithm, when disabled charger uses CC-CV algorithm 
output_Mode             = 1,           //   USER PARAMETER - 0 = PSU MODE, 1 = Charger Mode  
disableFlashAutoLoad    = 0,           //   USER PARAMETER - Forces the MPPT to not use flash saved settings, enabling this "1" defaults to programmed firmware settings.
enablePPWM              = 1,           //   USER PARAMETER - Enables Predictive PWM, this accelerates regulation speed (only applicable for battery charging application)
enableWiFi              = 1,           //   USER PARAMETER - Enable WiFi Connection
enableFan               = 1,           //   USER PARAMETER - Enable Cooling Fan
enableBluetooth         = 1,           //   USER PARAMETER - Enable Bluetooth Connection
enableLCD               = 1,           //   USER PARAMETER - Enable LCD display
enableLCDBacklight      = 1,           //   USER PARAMETER - Enable LCD display's backlight
overrideFan             = 0,           //   USER PARAMETER - Fan always on
enableDynamicCooling    = 0;           //   USER PARAMETER - Enable for PWM cooling control 
int
serialTelemMode         = 1,           //  USER PARAMETER - Selects serial telemetry data feed (0 - Disable Serial, 1 - Display All Data, 2 - Display Essential, 3 - Number only)
pwmResolution           = 11,          //  USER PARAMETER - PWM Bit Resolution 
pwmFrequency            = 39000,       //  USER PARAMETER - PWM Switching Frequency - Hz (For Buck)
temperatureFan          = 60,          //  USER PARAMETER - Temperature threshold for fan to turn on
temperatureMax          = 90,          //  USER PARAMETER - Overtemperature, System Shudown When Exceeded (deg C)
telemCounterReset       = 0,           //  USER PARAMETER - Reset Telem Data Every (0 = Never, 1 = Day, 2 = Week, 3 = Month, 4 = Year) 
errorTimeLimit          = 1000,        //  USER PARAMETER - Time interval for reseting error counter (milliseconds)  
errorCountLimit         = 5,           //  USER PARAMETER - Maximum number of errors  
millisRoutineInterval   = 250,         //  USER PARAMETER - Time Interval Refresh Rate For Routine Functions (ms)
millisSerialInterval    = 1,           //  USER PARAMETER - Time Interval Refresh Rate For USB Serial Datafeed (ms)
millisLCDInterval       = 1000,        //  USER PARAMETER - Time Interval Refresh Rate For LCD Display (ms)
millisWiFiInterval      = 2000,        //  USER PARAMETER - Time Interval Refresh Rate For WiFi Telemetry (ms)
millisLCDBackLInterval  = 2000,        //  USER PARAMETER - Time Interval Refresh Rate For WiFi Telemetry (ms)
backlightSleepMode      = 0,           //  USER PARAMETER - 0 = Never, 1 = 10secs, 2 = 5mins, 3 = 1hr, 4 = 6 hrs, 5 = 12hrs, 6 = 1 day, 7 = 3 days, 8 = 1wk, 9 = 1month
baudRate                = 500000;      //  USER PARAMETER - USB Serial Baud Rate (bps)

float 
voltageBatteryMax       = 27.3000,     //   USER PARAMETER - Maximum Battery Charging Voltage (Output V)
voltageBatteryMin       = 22.4000,     //   USER PARAMETER - Minimum Battery Charging Voltage (Output V)
currentCharging         = 30.0000,     //   USER PARAMETER - Maximum Charging Current (A - Output)
electricalPrice         = 9.5000;      //   USER PARAMETER - Input electrical price per kWh (Dollar/kWh,Euro/kWh,Peso/kWh)

bool
ADS1015_Mode            = 0;          //  CALIB PARAMETER - Use 1 for ADS1015 ADC model use 0 for ADS1115 ADC model
int
ADC_GainSelect          = 1,          //  CALIB PARAMETER - ADC Gain Selection (0→±6.144V 3mV/bit, 1→±4.096V 2mV/bit, 2→±2.048V 1mV/bit)
avgCountVS              = 3,          //  CALIB PARAMETER - Voltage Sensor Average Sampling Count (Recommended: 3)
avgCountCS              = 4,          //  CALIB PARAMETER - Current Sensor Average Sampling Count (Recommended: 4)
avgCountTS              = 500;        //  CALIB PARAMETER - Temperature Sensor Average Sampling Count
float
inVoltageDivRatio       = 16.0000,    //  CALIB PARAMETER - Input voltage divider sensor ratio (change this value to calibrate voltage sensor)
outVoltageDivRatio      = 16.0000,    //  CALIB PARAMETER - Output voltage divider sensor ratio (change this value to calibrate voltage sensor)
vOutSystemMax           = 50.0000,    //  CALIB PARAMETER - 
cOutSystemMax           = 50.0000,    //  CALIB PARAMETER - 
ntcResistance           = 9000.00,   //  CALIB PARAMETER - NTC temp sensor's resistance. Change to 10000.00 if you are using a 10k NTC
voltageDropout          = 1.0000,     //  CALIB PARAMETER - Buck regulator's dropout voltage (DOV is present due to Max Duty Cycle Limit)
voltageBatteryThresh    = 1.0000,     //  CALIB PARAMETER - Power cuts-off when this voltage is reached (Output V)
currentInAbsolute       = 31.0000,    //  CALIB PARAMETER - Maximum Input Current The System Can Handle (A - Input)
currentOutAbsolute      = 50.0000,    //  CALIB PARAMETER - Maximum Output Current The System Can Handle (A - Input)
PPWM_margin             = 99.5000,    //  CALIB PARAMETER - Minimum Operating Duty Cycle for Predictive PWM (%)
PWM_MaxDC               = 97.0000,    //  CALIB PARAMETER - Maximum Operating Duty Cycle (%) 90%-97% is good
efficiencyRate          = 1.0000,     //  CALIB PARAMETER - Theroretical Buck Efficiency (% decimal)
currentMidPoint         = 2.5250,     //  CALIB PARAMETER - Current Sensor Midpoint (V)
currentSens             = 0.0000,     //  CALIB PARAMETER - Current Sensor Sensitivity (V/A)
currentSensV            = 0.1850,     //  CALIB PARAMETER - Current Sensor Sensitivity (mV/A)
vInSystemMin            = 0.1000;     //  CALIB PARAMETER - 

bool
buckEnable            = 0,           // SYSTEM PARAMETER - Buck Enable Status
fanStatus             = 0,           // SYSTEM PARAMETER - Fan activity status (1 = On, 0 = Off)
bypassEnable          = 0,           // SYSTEM PARAMETER - 
chargingPause         = 0,           // SYSTEM PARAMETER - 
lowPowerMode          = 0,           // SYSTEM PARAMETER - 
buttonRightStatus     = 0,           // SYSTEM PARAMETER -
buttonLeftStatus      = 0,           // SYSTEM PARAMETER - 
buttonBackStatus      = 0,           // SYSTEM PARAMETER - 
buttonSelectStatus    = 0,           // SYSTEM PARAMETER -
buttonRightCommand    = 0,           // SYSTEM PARAMETER - 
buttonLeftCommand     = 0,           // SYSTEM PARAMETER - 
buttonBackCommand     = 0,           // SYSTEM PARAMETER - 
buttonSelectCommand   = 0,           // SYSTEM PARAMETER -
settingMode           = 0,           // SYSTEM PARAMETER -
setMenuPage           = 0,           // SYSTEM PARAMETER -
boolTemp              = 0,           // SYSTEM PARAMETER -
flashMemLoad          = 0,           // SYSTEM PARAMETER -  
confirmationMenu      = 0,           // SYSTEM PARAMETER -      
WIFI                  = 0,           // SYSTEM PARAMETER - 
BNC                   = 0,           // SYSTEM PARAMETER -  
REC                   = 0,           // SYSTEM PARAMETER - 
FLV                   = 0,           // SYSTEM PARAMETER - 
IUV                   = 0,           // SYSTEM PARAMETER - 
IOV                   = 0,           // SYSTEM PARAMETER - 
IOC                   = 0,           // SYSTEM PARAMETER - 
OUV                   = 0,           // SYSTEM PARAMETER - 
OOV                   = 0,           // SYSTEM PARAMETER - 
OOC                   = 0,           // SYSTEM PARAMETER - 
OTE                   = 0;           // SYSTEM PARAMETER - 
int
inputSource           = 0,           // SYSTEM PARAMETER - 0 = MPPT has no power source, 1 = MPPT is using solar as source, 2 = MPPTis using battery as power source
avgStoreTS            = 0,           // SYSTEM PARAMETER - Temperature Sensor uses non invasive averaging, this is used an accumulator for mean averaging
temperature           = 0,           // SYSTEM PARAMETER -
sampleStoreTS         = 0,           // SYSTEM PARAMETER - TS AVG nth Sample
pwmMax                = 0,           // SYSTEM PARAMETER -
pwmMaxLimited         = 0,           // SYSTEM PARAMETER -
PWM                   = 0,           // SYSTEM PARAMETER -
PPWM                  = 0,           // SYSTEM PARAMETER -
pwmChannel            = 0,           // SYSTEM PARAMETER -
batteryPercent        = 0,           // SYSTEM PARAMETER -
errorCount            = 0,           // SYSTEM PARAMETER -
menuPage              = 0,           // SYSTEM PARAMETER -
subMenuPage           = 0,           // SYSTEM PARAMETER -
ERR                   = 0,           // SYSTEM PARAMETER - 
conv1                 = 0,           // SYSTEM PARAMETER -
conv2                 = 0,           // SYSTEM PARAMETER -
intTemp               = 0;           // SYSTEM PARAMETER -
float
VSI                   = 0.0000,      // SYSTEM PARAMETER - Raw input voltage sensor ADC voltage
VSO                   = 0.0000,      // SYSTEM PARAMETER - Raw output voltage sensor ADC voltage
CSI                   = 0.0000,      // SYSTEM PARAMETER - Raw current sensor ADC voltage
CSI_converted         = 0.0000,      // SYSTEM PARAMETER - Actual current sensor ADC voltage 
TS                    = 0.0000,      // SYSTEM PARAMETER - Raw temperature sensor ADC value
powerInput            = 0.0000,      // SYSTEM PARAMETER - Input power (solar power) in Watts
powerInputPrev        = 0.0000,      // SYSTEM PARAMETER - Previously stored input power variable for MPPT algorithm (Watts)
powerOutput           = 0.0000,      // SYSTEM PARAMETER - Output power (battery or charing power in Watts)
energySavings         = 0.0000,      // SYSTEM PARAMETER - Energy savings in fiat currency (Peso, USD, Euros or etc...)
voltageInput          = 0.0000,      // SYSTEM PARAMETER - Input voltage (solar voltage)
voltageInputPrev      = 0.0000,      // SYSTEM PARAMETER - Previously stored input voltage variable for MPPT algorithm
voltageOutput         = 0.0000,      // SYSTEM PARAMETER - Input voltage (battery voltage)
currentInput          = 0.0000,      // SYSTEM PARAMETER - Output power (battery or charing voltage)
currentOutput         = 0.0000,      // SYSTEM PARAMETER - Output current (battery or charing current in Amperes)
TSlog                 = 0.0000,      // SYSTEM PARAMETER - Part of NTC thermistor thermal sensing code
ADC_BitReso           = 0.0000,      // SYSTEM PARAMETER - System detects the approriate bit resolution factor for ADS1015/ADS1115 ADC
daysRunning           = 0.0000,      // SYSTEM PARAMETER - Stores the total number of days the MPPT device has been running since last powered
Wh                    = 0.0000,      // SYSTEM PARAMETER - Stores the accumulated energy harvested (Watt-Hours)
kWh                   = 0.0000,      // SYSTEM PARAMETER - Stores the accumulated energy harvested (Kiliowatt-Hours)
MWh                   = 0.0000,      // SYSTEM PARAMETER - Stores the accumulated energy harvested (Megawatt-Hours)
loopTime              = 0.0000,      // SYSTEM PARAMETER -
outputDeviation       = 0.0000,      // SYSTEM PARAMETER - Output Voltage Deviation (%)
buckEfficiency        = 0.0000,      // SYSTEM PARAMETER - Measure buck converter power conversion efficiency (only applicable to my dual current sensor version)
floatTemp             = 0.0000,
vOutSystemMin         = 0.0000;     //  CALIB PARAMETER - 
unsigned long 
currentErrorMillis    = 0,           //SYSTEM PARAMETER -
currentButtonMillis   = 0,           //SYSTEM PARAMETER -
currentSerialMillis   = 0,           //SYSTEM PARAMETER -
currentRoutineMillis  = 0,           //SYSTEM PARAMETER -
currentLCDMillis      = 0,           //SYSTEM PARAMETER - 
currentLCDBackLMillis = 0,           //SYSTEM PARAMETER - 
currentWiFiMillis     = 0,           //SYSTEM PARAMETER - 
currentMenuSetMillis  = 0,           //SYSTEM PARAMETER - 
prevButtonMillis      = 0,           //SYSTEM PARAMETER -
prevSerialMillis      = 0,           //SYSTEM PARAMETER -
prevRoutineMillis     = 0,           //SYSTEM PARAMETER -
prevErrorMillis       = 0,           //SYSTEM PARAMETER -
prevWiFiMillis        = 0,           //SYSTEM PARAMETER -
prevLCDMillis         = 0,           //SYSTEM PARAMETER -
prevLCDBackLMillis    = 0,           //SYSTEM PARAMETER -
timeOn                = 0,           //SYSTEM PARAMETER -
loopTimeStart         = 0,           //SYSTEM PARAMETER - Used for the loop cycle stop watch, records the loop start time
loopTimeEnd           = 0,           //SYSTEM PARAMETER - Used for the loop cycle stop watch, records the loop end time
secondsElapsed        = 0;           //SYSTEM PARAMETER - 

//================= CORE0: SETUP (DUAL CORE MODE) =====================//
void coreTwo(void * pvParameters){
  // Kết nối WiFi
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi: ");
  WiFi.begin(ssid, password);
  Serial.println(ssid);
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(">");
  }
  Serial.println("CONNECTED !!!");   
  Time = millis();                                      
  
//================= CORE0: LOOP (DUAL CORE MODE) ======================//
  while(1){

     if ((unsigned long)millis() - Time > 58000)
  {
    Serial.println("Sending data at: " + String(millis()));
    print_data();
    Time = millis();
    Serial.println("Data sent at: " + String(millis())); 
  }
    
}}
//================== CORE1: SETUP (DUAL CORE MODE) ====================//
void setup() { 
  
  //SERIAL INITIALIZATION          
  Serial.begin(baudRate);                                   //Set serial baud rate
  Serial.println("> Serial Initialized");                   //Startup message
  
  //GPIO PIN INITIALIZATION
  pinMode(backflow_MOSFET,OUTPUT); 
  pinMode(backflow_BAT,OUTPUT);
  digitalWrite(backflow_MOSFET,0);                          //Signal backflow MOSFET GPIO pin
  digitalWrite(backflow_BAT,1);                          
  pinMode(buck_EN,OUTPUT);
  pinMode(LED,OUTPUT); 
  pinMode(FAN,OUTPUT);
  pinMode(TS,INPUT); 
  pinMode(ADC_ALERT,INPUT);
  pinMode(buttonLeft,INPUT); 
  pinMode(buttonRight,INPUT); 
  pinMode(buttonBack,INPUT); 
  pinMode(buttonSelect,INPUT); 
  
  //PWM INITIALIZATION
  ledcSetup(pwmChannel,pwmFrequency,pwmResolution);          //Set PWM Parameters
  ledcAttachPin(buck_IN, pwmChannel);                        //Set pin as PWM
  ledcWrite(pwmChannel,PWM);                                 //Write PWM value at startup (duty = 0)
  pwmMax = pow(2,pwmResolution)-1;                           //Get PWM Max Bit Ceiling
  pwmMaxLimited = (PWM_MaxDC*pwmMax)/100.000;                //Get maximum PWM Duty Cycle (pwm limiting protection)
  
  //ADC INITIALIZATION
  ADC_SetGain();                                             //Sets ADC Gain & Range
  ads.begin();                                               //Initialize ADC

  //GPIO INITIALIZATION                          
  buck_Disable();

  //ENABLE DUAL CORE MULTITASKING
  xTaskCreatePinnedToCore(coreTwo,"coreTwo",10000,NULL,0,&Core2,0);
  
  //INITIALIZE AND LIOAD FLASH MEMORY DATA
  EEPROM.begin(512);
  Serial.println("> FLASH MEMORY: STORAGE INITIALIZED");  //Startup message 
  initializeFlashAutoload();                              //Load stored settings from flash memory       
  Serial.println("> FLASH MEMORY: SAVED DATA LOADED");    //Startup message 

  //LCD INITIALIZATION
  if(enableLCD==1){
    lcd.begin();
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);
    lcd.print("MPPT INITIALIZED");
    lcd.setCursor(0,1);
    lcd.print("FIRMWARE ");
    lcd.print(firmwareInfo);    
    delay(500);
    lcd.clear();
  }

  //SETUP FINISHED
  Serial.println("> MPPT HAS INITIALIZED");                //Startup message

}
//================== CORE1: LOOP (DUAL CORE MODE) ======================//
void loop() {
  Read_Sensors();         //TAB#2 - Sensor data measurement and computation
  Device_Protection();    //TAB#3 - Fault detection algorithm  
  System_Processes();     //TAB#4 - Routine system processes 
  Charging_Algorithm();   //TAB#5 - Battery Charging Algorithm                    
  LCD_Menu();             //TAB#8 - Low Power Algorithm
}
