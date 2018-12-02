#include <ArduinoJson.h>
#include <OneWire.h> // OneWire kütüphanesini ekliyoruz.
#include <DallasTemperature.h>
#include <NewPing.h>
#include <Wire.h>
#include <stdio.h>
#include <DS1302.h>
#include <EEPROM.h>


/* global variables (TO BE SENT) */
String IMEI;
//int tank_id = 2;
String last_seen;
float temp = 30;
float battery = 12;
int water_level_percentage = 100;
float chlorine_level = 0.25;
int chlorine_tank_level_percentage = 100;
float ph_level = 7.35;

/* global variables (TO BE RECEIVED) */
unsigned long get_interval = 20000;
unsigned long get_interval_counter = 0;
int delay_chunk = 1000;
unsigned long post_interval = 30000;
unsigned long post_interval_counter = 0;
bool pump_state = true;
String rtc_time = "00:00:00";

/*** DEFINITIONS ***/
// ###PUMP PIN###
#define PUMP_PIN 2

// ###PIR Dedektör###
#define SERIAL 12
#define DL 53
#define BUZZER 29
#define INTER 3
volatile int intruder_flag = 0;

// ###AKÜ###
// float battery = 12;

// ###Sıcaklık###
#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// float temp = 300;

// ###WATER_LEVEL_PERCENTAGE###
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_WATER_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_WATER_DISTANCE); // NewPing setup of pins and maximum distance.
// int water_level_percentage = 100;

// ###CHLORINE_TANK_LEVEL_PERCENTAGE###
#define TRIGGER_PIN1 9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1    8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_CHLORINE_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_CHLORINE_DISTANCE); // NewPing setup of pins and maximum distance.
// int chlorine_tank_level_percentage = 100;

// ###CHLORINE_LEVEL###
// ORP Okuma İşlemi için gerekli tanımlamalar
#define i2c_id 0x66                     //default I2C address   
#define one_byte_read 0x01              //used in a function to read data from the device  
#define two_byte_read 0x02              //used in a function to read data from the device
#define four_byte_read 0x04             //used in a function to read data from the device
byte bus_address = i2c_id;              //holds the I2C address. 
byte bytes_received_from_computer = 0;  //we need to know how many character bytes have been received.
byte serial_event = 0;                  //a flag to signal when data has been received from the serial monitor
char computerdata[20];                  //we make an 20 byte character array to hold incoming data from the serial monitor
char *cmd;                              //char pointer used in string parsing
char *data_byte_1;                      //char pointer used in string parsing
char *data_byte_2;                      //char pointer used in string parsing
union sensor_mem_handler                //declare the use of a union data type
{
  byte i2c_data[4];                   //define a 4 byte array in the union
   long answ;             //define an long in the union
};
union sensor_mem_handler move_data;     //declare that we will refer to the union as �move_data�
volatile byte new_reading = 0;          //a flag to signal when the interrupt pin sees a new reading 
volatile byte continuous_mode = 0;      //use to enable / disable continuous readings 
float orp;
// float chlorine_level;

// ###RTCTIME###
namespace {
const int kCePin   = 37;  // Chip Enable
const int kIoPin   = 31;  // Input/Output
const int kSclkPin = 35;  // Serial Clock
// Create a DS1302 object.
DS1302 rtc(kCePin, kIoPin, kSclkPin);
}
/*******************/


void setup() {
  
  // Start Serial
  Serial.begin(9600);           //enable serial port.
  ShowSerialData();

  // Define PIR Pins
  pinMode(BUZZER, OUTPUT);
  pinMode(SERIAL,OUTPUT);
  pinMode(DL,INPUT);
  pinMode(INTER,INPUT);
  pinMode(BUZZER,OUTPUT);
  digitalWrite(BUZZER,LOW);
  attachInterrupt(digitalPinToInterrupt(INTER), ISR2, RISING);
  Serial.print("initializing PIR detector...");
  writeregval(SERIAL,0x00304D10);
  Serial.print("success.\n");

  // Start I2C
  Wire.begin();

  // Activate Pump
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(2, LOW);
  
  // Activate GSM
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(23, LOW);
  digitalWrite(22, LOW);
  digitalWrite(23, HIGH);

  // Start Serial1 (GSM)
  delay(60000);
  Serial1.begin(19200);
  delay(20000);

  /*printDataParams();
  Serial.print("____");
  readValues();
  Serial.print("____");
  printDataParams();*/
 
  Serial.println("fetching parameters...");
  String httpResponse = SubmitHTTPRequest();
  adjustParameters(httpResponse);
}


void loop() {
  Serial.println("Starting loop.");
  String httpResponse = SubmitHTTPRequest();
  Serial.println("server responded: ");
  Serial.println(httpResponse);
  adjustParameters(httpResponse);
  
  //delay(get_interval * 60 * 1000);
  //delay(get_interval);
  get_interval_counter = 0;
  while (get_interval_counter < get_interval) 
  {
    // check intruder_flag and set last seen here
    Serial.println("Counting... GIC :" + String(get_interval_counter) + ", GI :" + String(get_interval));
    if (intruder_flag)
    {
      intruder_flag = 0;
      Serial.println("Intruder detected! Triggering buzzer.");
      delay(10);

      digitalWrite(BUZZER,LOW);

      pinMode(INTER,OUTPUT);
      digitalWrite(INTER,LOW);

      Serial.println("buzzer delay starts");
      _delay_ms(2000);
      pinMode(INTER,INPUT);
      Serial.println("buzzer delay ends");
     
      Serial.println("Terminating buzzer.");
      printDataParams();
      Serial.println("____values before buzzer____");
      readValues();
      Serial.println("____values after buzzer____");
      printDataParams();
      attachInterrupt(digitalPinToInterrupt(INTER), ISR2, RISING);
      //EKLEMELER YAPILABİLİR
    }
    delay(delay_chunk);
    get_interval_counter += delay_chunk;
  }
  post_interval_counter += get_interval;
}

void getIMEI()
{
  Serial1.println("AT+GSN");
  delay(500);
  while (Serial1.available()!=0) {
    char nextChar = char(Serial1.read());
    int nextCharASCII = int (nextChar);
    if (nextCharASCII >= 48 && nextCharASCII <= 57) {
      IMEI += nextChar;
    }
  }
  Serial.println("received IMEI: \"" + IMEI + "\"");
}

String SubmitHTTPRequest()
{
  Serial1.println("AT+CSQ"); // Signal quality check ([?] gerek var mi?)
  delay(100);
  //flushSerial1Buffer();
  ShowSerialData();
  
  Serial1.println("AT+CGATT=1"); // Attach to GRPS Support ([?] bu =? şeklindeydi. olanı soruyordu. =1e çektim)
  flushSerial1Buffer();
  delay(100);
  ShowSerialData();

  Serial1.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""); // Connection type = GRPS
  delay(1000);
  ShowSerialData();

  Serial1.println("AT+SAPBR=3,1,\"APN\",\"internet\""); // Setup APN
  delay(1000);
  ShowSerialData();

  Serial1.println("AT+SAPBR=1,1"); // Enable GPRS connection
  delay(1000);
  ShowSerialData();
  
  Serial1.println("AT+SAPBR=2,1"); // Get IP
  delay(1000);
  ShowSerialData();

  Serial1.println("AT+HTTPINIT"); // Initialize HTTP Service
  delay(1000);
  ShowSerialData();

  if (IMEI == "")
  {
    Serial.print("IMEI not received! Fetching IMEI...");
    getIMEI();  
  }
  
  Serial1.println("AT+HTTPPARA=\"CID\",1"); // Set parameters for HTTP Session
  delay(1000);
  ShowSerialData();

  Serial1.println("AT+HTTPPARA=\"REDIR\",1"); // Auto redirect
  delay(1000);
  ShowSerialData();

  if (post_interval_counter < post_interval) // Get Request
  {
    Serial.println("sending get request! counter: " + String(post_interval_counter) + ", interval: " + String(post_interval));
    Serial1.println("AT+HTTPPARA=\"URL\",\"api.bttgsutakip.com:6792/tank/" + IMEI + "\""); // Set URL 
    delay(1000); 
    ShowSerialData();
    Serial1.println("AT+HTTPACTION=0"); // Submit GET request (Action=0)  
    delay(10000);
    ShowSerialData();
  }
  else // POST Request
  {
    post_interval_counter = 0;
    Serial.print("Reading values before POSTRequest...\n");
    printDataParams();
    Serial.print("____");
    readValues();
    Serial.print("____");
    printDataParams();
    Serial.println("sending POST REQUEST!!");
    Serial1.println("AT+HTTPPARA=\"CONTENT\",\"application/json\""); // Set header for POST request 
    delay(1000);
    ShowSerialData();
    Serial1.println("AT+HTTPPARA=\"URL\",\"api.bttgsutakip.com:6792/tank\""); // Set URL
    delay(1000);
    ShowSerialData();
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["IMEI"] = IMEI;
    root["temp"] = temp;
    root["battery"] = battery;
    root["water_level_percentage"] = water_level_percentage;
    root["chlorine_level"] = chlorine_level;
    root["chlorine_tank_level_percentage"] = chlorine_tank_level_percentage;
    root["ph_level"] = ph_level;
    root["last_seen"] = last_seen;
    String output;
    root.printTo(output);
    Serial.print("sending output: _" + output + "-\n");
    String command = "AT+HTTPDATA=" + String(output.length()) + ",9500";
    Serial.print("command: " + command);
    Serial1.println(command); // Data size = 1500byte, Maximum time allowed=20000ms
    //Serial1.println("AT+HTTPDATA=2500,5000");
    delay(5000);
    ShowSerialData();
    Serial1.println(output);
    delay(5000);
    ShowSerialData();
    Serial1.println("AT+HTTPACTION=1"); // Submit POST request (Action=1)
    delay(20000);
    ShowSerialData();
    // todo: form JSON and attach to HTTPRequest here.
  }
  Serial.print("reading response...");
  Serial1.println("AT+HTTPREAD");// Read response
  delay(20000);
  String response = "";
  while (Serial1.available()!=0) {
    response+=char(Serial1.read());
  }
  Serial.print("response starts");
  Serial.print(response);
  Serial.print("response ends");
  return response;
}

void adjustParameters(String httpResponse)
{
  Serial.println("Adjusting parameters... INITIAL VALUES:");
  printParams();
  StaticJsonBuffer<200> jsonBuffer;
  String jsonString = getJSONString(httpResponse);
  Serial.println("API responds:" + jsonString);
  Serial.print("Converting response to JSON and applying changes...");
  char jsonCharArray[jsonString.length()+1];
  jsonString.toCharArray(jsonCharArray, jsonString.length()+1);
  JsonObject& root = jsonBuffer.parseObject(jsonCharArray);
  
  pump_state = root["pump"];
  unsigned long getint = root["get"];
  unsigned long postint = root["post"];
  if (getint == 0) { getint = 20000; pump_state=true; }
  if (postint == 0) { postint = 30000; pump_state=true; }
  get_interval = getint;
  post_interval = postint;
  const char* temp_rtc_time = root["time"];
  rtc_time = String(temp_rtc_time);
  setTime(rtc_time);
  setPump();
  Serial.print(" [OK]\n");
  Serial.println("Successfully adjusted parameters. FINAL VALUES:"); 
  printParams();
}


void ShowSerialData()
{
  while (Serial1.available() != 0)
  {
    Serial.write(char (Serial1.read()));
    //Serial1.read();
  }
}

void flushSerial1Buffer()
{
  while (Serial1.available() != 0)
  { Serial1.read(); }
}

String getJSONString(String HTTPResponse) 
{
  return HTTPResponse.substring(HTTPResponse.indexOf('{'), HTTPResponse.indexOf('}')+1);
}

void printParams()
{
  Serial.println("[GET_INTERVAL: " + String(get_interval) + "]");
  Serial.println("[POST_INTERVAL: " + String(post_interval) + "]");
  Serial.println("[PUMP_STATE: " + String(pump_state) + "]");
  Serial.println("[RTCTIME: " + String(rtc_time) + "]");
}

void printDataParams()
{
  Serial.println("[BATTERY: " + String(battery) + "]");
  Serial.println("[TEMP: " + String(temp) + "]");
  Serial.println("[WATER_LEVEL_PERCENTAGE: " + String(water_level_percentage) + "]");
  Serial.println("[CHLORINE_LEVEL: " + String(chlorine_level) + "]");
  Serial.println("[CHLORINE_TANK_LEVEL_PERCENTAGE: " + String(chlorine_tank_level_percentage) + "]");
  Serial.println("[PH_LEVEL: " + String(ph_level) + "]");
}

void readValues() 
{
  readBattery();
  readTemp();
  readWaterLevelPercentage();
  readChlorineTankLevelPercentage();
  readChlorineLevel();
}


void ISR2()
{
  Serial.println("###ISR2 PING!!!!");
  detachInterrupt(digitalPinToInterrupt(INTER));
  digitalWrite(BUZZER, HIGH);
  intruder_flag = 1;

  Time t = rtc.time();

  // Name the day of the week.
  const String day = dayAsString(t.day);

  // Format the time and date and insert into the temporary buffer.
  char buf[50];
  snprintf(buf, sizeof(buf), "%s %04d-%02d-%02d %02d:%02d:%02d",
           day.c_str(),
           t.yr, t.mon, t.date,
           t.hr, t.min, t.sec);
  last_seen = String(buf);
}

void writeregval(int pin1, unsigned long regval) {
  int i;
  int _pin1=pin1;
  unsigned long _regval=regval;
  unsigned char nextbit;
  unsigned long regmask = 0x1000000;
  pinMode(_pin1,OUTPUT);
  digitalWrite(_pin1,LOW);
  for(i=0;i < 25;i++){
    nextbit = (_regval&regmask)!=0;
    regmask >>=1;
    digitalWrite(_pin1,LOW);
    digitalWrite(_pin1,HIGH);
    
    if(nextbit){
      digitalWrite(_pin1,HIGH);
      Serial.print("1");}
    else{
      digitalWrite(_pin1,LOW);
      Serial.print("0");}
    _delay_us(100);
  }

  digitalWrite(_pin1,LOW);
  _delay_us(600);
  pinMode(INTER,OUTPUT);
  digitalWrite(INTER,LOW);
  _delay_ms(2000);
  pinMode(INTER,INPUT);
  Serial.print("\ndone\n");
}


void readBattery() 
{
  battery = analogRead(A0)*57.1/1024;
}

void readTemp()
{
  int y = 0;
  while (y<2)
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
    //Serial.println(sensors.getTempCByIndex(0)); // Why "byIndex"? 
    temp = sensors.getTempCByIndex(0);
    delay(1000);
    y++;  
  }
}

void readWaterLevelPercentage()
{
  int registered_trials_count = 0;
  int trials_count = 0;
  int percentage_sum = 0;
  int temp_percentage;
  while (trials_count<100 && registered_trials_count<10) 
  {
    float water_level_in_cm = sonar.ping_cm();
    temp_percentage = 100 * (MAX_WATER_DISTANCE-int(water_level_in_cm))/MAX_WATER_DISTANCE;
    if (temp_percentage<=0) { Serial.println("[WLP TRIAL " + String(trials_count) + "] 0- (ignored)"); trials_count++; continue; }
    if (temp_percentage>=100) { Serial.println("[WLP TRIAL " + String(trials_count) + "] 100+ (ignored)"); trials_count++; continue; };
    Serial.println("[WLP TRIAL " + String(trials_count) + "] " + String(temp_percentage) + " (registered)");
    percentage_sum += temp_percentage;
    registered_trials_count += 1;
    trials_count += 1;
    delay(100);  
  }
  if (registered_trials_count == 0) { 
    water_level_percentage = 100;
    Serial.println("No successful trials were registered. Returning water_level_percentage=100");
  } 
  else {
    water_level_percentage = int(percentage_sum / registered_trials_count);
    Serial.println("Total registered calculations: " + String(registered_trials_count) + ", Average: " + String(water_level_percentage));
  }
}

void readChlorineTankLevelPercentage()
{
  int registered_trials_count = 0;
  int trials_count = 0;
  int percentage_sum = 0;
  int temp_percentage;
  while (trials_count<100 && registered_trials_count<10) 
  {
    float chlorine_level_in_cm = sonar1.ping_cm();
    temp_percentage = 100 * (MAX_CHLORINE_DISTANCE-int(chlorine_level_in_cm))/MAX_CHLORINE_DISTANCE;
    if (temp_percentage<=0) { Serial.println("[CLP TRIAL " + String(trials_count) + "] 0- (ignored)"); trials_count++; continue; }
    if (temp_percentage>=100) { Serial.println("[CLP TRIAL " + String(trials_count) + "] 100+ (ignored)"); trials_count++; continue; };
    Serial.println("[CLP TRIAL " + String(trials_count) + "] " + String(temp_percentage) + " (registered)");
    percentage_sum += temp_percentage;
    registered_trials_count += 1;
    trials_count += 1;
    delay(100);
  }
  if (registered_trials_count == 0) { 
    chlorine_tank_level_percentage = 100;
    Serial.println("No successful trials were registered. Returning chlorine_tank_level_percentage=100");
  } 
  else {
    chlorine_tank_level_percentage = int(percentage_sum / registered_trials_count);
    Serial.println("Total registered calculations: " + String(registered_trials_count) + ", Average: " + String(chlorine_tank_level_percentage));
  }
}


void parse_data() {                     //once a string is received from the serial monitor it is parsed at each comma
  byte i;                                                 //counter

  for (i = 0; i < bytes_received_from_computer; i++) {    //for each char byte received 
    computerdata[i] = tolower(computerdata[i]);     //set the char to lower case 
  }

  cmd = strtok(computerdata, ",");                        //let's parse the string at each comma
  data_byte_1 = strtok(NULL, ",");                        //let's parse the string at each comma
  data_byte_2 = strtok(NULL, ",");                        //let's parse the string at each comma
}
//*************************************************************************************************************************
//*************************************************************************************************************************

void read_command() {                   //we evaluate the string stored in the �cmd� var 

  if (strcasecmp(cmd, "?") == 0) {                        //if the command is: ?  
    explain_commands();                 //call function: "explain_commands"
  }
  if (strcmp(cmd, "i") == 0) {                            //if the command is: i  
    //info();                                             //call function:"info"
  }
  if (strcmp(cmd, "adr") == 0) {                          //if the command is: adr
    //adress_con();                                       //call function:"adress_con"
  }
  if (strcmp(cmd, "int") == 0) {                          //if the command is: int 
    //int_con();                                          //call function:"int_con"
  }
  if (strcmp(cmd, "led") == 0) {                          //if the command is: led
    //led_con();                                          //call function:"led_con"
  }
  if ((strcmp(cmd, "on") == 0) || (strcmp(cmd, "off") == 0)) {//if the command is: on or off
    //active_con();                                       //call function:"active_con"
  }
  if (strcmp(cmd, "nra") == 0) {                          //if the command is: nra
    //nra();                                               //call function:"nra"
  }
  if (strcmp(cmd, "cal") == 0) {                          //if the command is: cal
    calibration();                                      //call function:"calibration"
  } 
  if (strcmp(cmd, "r") == 0) {                            //if the command is: r

    if (strcmp(data_byte_1, "1") == 0) {                //if the command is: r,1
      continuous_mode = 1;                            //set the var continuous_mode = 1 
      Serial.println("continuous mode on");
    }
    if (strcmp(data_byte_1, "0") == 0) {                //if the command is: r,0
      continuous_mode = 0;                            //set the var continuous_mode = 0
      Serial.println("continuous mode off");
    }
    reading();                      //call function:"reading" 
  }
}
//*************************************************************************************************************************
//*************************************************************************************************************************

void explain_commands() {                               //if the command is: ? a list of all possible commands will be printed  

  Serial.println(F("**commands are not case sensitive**"));
  Serial.println();
  Serial.println(F("i = deice type and version number"));
  Serial.println();
  Serial.println(F("adr,? = what is the I2C ID number"));
  Serial.println(F("adr,unlock = unlock the I2C address change register"));
  Serial.println(F("adr,lock = Lock the address change register"));
  Serial.println(F("adr,new,[new i2c address number]"));
  Serial.println();
  Serial.println(F("int,? =  read the state of the interrupt control register"));
  Serial.println(F("int,[high],[low],[inv],[off] = set the interrupt control register"));
  Serial.println();
  Serial.println(F("led,? = read the state of the LED control register"));
  Serial.println(F("led,[on],[off] = set the LED control register"));
  Serial.println();
  Serial.println(F("on = start taking readings"));
  Serial.println(F("off = stop taking readings; hibernate"));
  Serial.println();
  Serial.println(F("nra,? = read the state of the new reading available register"));
  Serial.println(F("nra,clr = clear the new reading available register"));
  Serial.println();
  Serial.println(F("cal,? = read the state of the calibration register"));
  Serial.println(F("cal,XXX.X = calibrate to value"));
  Serial.println(F("cal,clr = clear the calibration"));
  Serial.println();
  Serial.println(F("r = take a single ORP reading "));
  Serial.println(F("r,1 = take continuous readings"));
  Serial.println(F("r,0 = END continuous readings"));
}

void calibration() {                          //if the command is: cal
                                    //cal,? = read the state of the calibration control register
                                    //cal,XXX.X = calibrate to value 
                                    //cal,clr = clear the calibration

  const byte calibration_value_register = 0x08;           //register to read / write
  const byte calibration_request_register = 0x0C;           //register to read / write
  const byte calibration_confirmation_register = 0x0D;                //register to read
  const byte cal_clear = 0x01;                                        //clear calibration
  const byte calibrate = 0x02;                    //calibrate to value
  float calibration = 0;                                              //used to hold the new calibration value 


  if (strcmp(data_byte_1, "?") == 0) {                //if the command sent was: "cal,?"
    Serial.print("calibration status: ");
    i2c_read(calibration_confirmation_register, one_byte_read);     //I2C_read(OEM register, number of bytes to read) and print calibration state
    if (move_data.i2c_data[0] == 0)Serial.println("no calibration");
    if (move_data.i2c_data[0] == 1)Serial.println("calibrated");
  }

  if (strcmp(data_byte_1, "clr") == 0) {                //if the command sent was: "cal,clr"
    i2c_write_byte(calibration_request_register, cal_clear);    //write the calibration clear command to the calibration control register  
    delay(40);                            //wait for the calibration event to finish
    i2c_read(calibration_confirmation_register, one_byte_read);   //read from the calibration control register to confirm it is set correctly 
    if (move_data.i2c_data[0] == 0)Serial.println("calibration cleard 0");
  }
  
  if (isdigit(computerdata[5])){                                //if the command sent was: "cal,xxxx" where x is a digit
    calibration = atof(data_byte_1);                //convert the calibration value entered from a string to a float
    calibration *= 10;                        //multiply by 10 to remove the decimal point    
    move_data.answ = calibration;                 //move the float to a long 
    i2c_write_long(calibration_value_register, move_data.answ);   //write the 4 bytes of the long to the calibration register  
    i2c_write_byte(calibration_request_register, calibrate);    //write the calibration command to the calibration control register  
    delay(10);                            //wait for the calibration event to finish 
    i2c_read(calibration_confirmation_register, one_byte_read);   //read from the calibration control register to confirm it is set correctly
    if (move_data.i2c_data[0] == 1) Serial.println("calibrated");
  }
}
//*************************************************************************************************************************
//*************************************************************************************************************************

void reading() {                  //if the command is: r
                                  //r = take a single
                                  //r,1 = take a reading one after the other **interrupt control must be set to "inv" first
                                  //r,0 = stop taking readings one after the other    

  const byte orp_register = 0x0E;                 //register to read
  orp = 0;                          //used to hold the new ORP value

  i2c_read(orp_register, four_byte_read);             //I2C_read(OEM register, number of bytes to read)                   
  orp = move_data.answ;                     //move the 4 bytes read into a float
  orp /= 10;                            //divide by 10 to get the decimal point 
  //Serial.print("ORP= ");
  Serial.println(orp);                                          //print info from register block
  
}

//*************************************************************************************************************************
//*************************************************************************************************************************

void i2c_read(byte reg, byte number_of_bytes_to_read) {                       //used to read 1,2,and 4 bytes: i2c_read(starting register,number of bytes to read)    

  byte i;                                             //counter

  Wire.beginTransmission(bus_address);                              //call the device by its ID number
  Wire.write(reg);                                        //transmit the register that we will start from
  Wire.endTransmission();                                     //end the I2C data transmission
  Wire.requestFrom(bus_address, (byte)number_of_bytes_to_read);                 //call the device and request to read X bytes
  for (i = number_of_bytes_to_read; i>0; i--) { move_data.i2c_data[i - 1] = Wire.read(); }        //with this code we read multiple bytes in reverse
  Wire.endTransmission();                                     //end the I2C data transmission  
}
//*************************************************************************************************************************
//*************************************************************************************************************************

void i2c_write_byte(byte reg, byte data) {                              //used to write a single byte to a register: i2c_write_byte(register to write to, byte data) 

  Wire.beginTransmission(bus_address);                              //call the device by its ID number
  Wire.write(reg);                                        //transmit the register that we will start from
  Wire.write(data);                                       //write the byte to be written to the register 
  Wire.endTransmission();                                 //end the I2C data transmission
}
//*************************************************************************************************************************
//*************************************************************************************************************************

void i2c_write_long(byte reg, long data) {                              //used to write a 4 bytes to a register: i2c_write_long(register to start at, long data )                     

  int i;                                                                                          //counter

  Wire.beginTransmission(bus_address);                              //call the device by its ID number
  Wire.write(reg);                                        //transmit the register that we will start from
  for (i = 3; i >= 0; i--) {                                    //with this code we write multiple bytes in reverse
    Wire.write(move_data.i2c_data[i]);
  }
  Wire.endTransmission();                                                                  //end the I2C data transmission
}
//*************************************************************************************************************************
//*************************************************************************************************************************
void active_con() {
  //if the command is: act
  //act,? = read the state of the active / hibernate control register
  //act,[on],[off] = take readings  
  const byte active_hibernate_register = 0x06;                    //register to read / write
  const byte active_mode = 0x01;                                  //take readings
  const byte hibernate_mode = 0x00;                               //stop taking readings
  if (strcmp(cmd, "on") == 0) {                 //if the command sent was: "on"
    i2c_write_byte(active_hibernate_register, active_mode);     //write the active mode enable command
    i2c_read(active_hibernate_register, one_byte_read);         //read from the active / hibernate control register to confirm it is set correctly 
    if (move_data.i2c_data[0] == 1) {
  //   Serial.println("active");
    }
  }

  if (strcmp(cmd, "off") == 0) {                  //if the command sent was: "off"
    i2c_write_byte(active_hibernate_register, hibernate_mode);  //write the active mode disable command
    i2c_read(active_hibernate_register, one_byte_read);     //read from the active / hibernate control register to confirm it is set correctly 
    if (move_data.i2c_data[0] == 0) {
    //  Serial.println("hibernate");
    }
  }
}

void readChlorineLevel()
{
  cmd = "on";
  active_con();
  delay(1000);
  reading();
  cmd = "off";
  active_con();
  chlorine_level = orp / 1000;
}


String dayAsString(const Time::Day day)
{
  switch (day) {
    case Time::kSunday: return "Pazar";
    case Time::kMonday: return "Pazartesi";
    case Time::kTuesday: return "Sal";
    case Time::kWednesday: return "Carsamba";
    case Time::kThursday: return "Persembe";
    case Time::kFriday: return "Cuma";
    case Time::kSaturday: return "Cumartesi";
  }
  return "(unknown day)";
}

Time::Day indexAsDay(int dayIndex)
{
  Serial.print("input indexAsDay: _" + String(dayIndex) + "-");
  switch (dayIndex) {
    case 1: Serial.print("0ping"); return Time::kMonday;
    case 2: Serial.print("0ping"); return Time::kTuesday;
    case 3: Serial.print("0ping"); return Time::kWednesday;
    case 4: Serial.print("0ping"); return Time::kThursday;
    case 5: Serial.print("0ping"); return Time::kFriday;
    case 6: Serial.print("0ping"); return Time::kSaturday;
    case 0: Serial.print("0ping"); return Time::kSunday;
  }
  Serial.println("[ERROR@stringAsDay] returning default value (Sunday).");
  return Time::kSunday;
}

void printTime()
{
  // Get the current time and date from the chip.
  Time t = rtc.time();

  // Name the day of the week.
  const String day = dayAsString(t.day);

  // Format the time and date and insert into the temporary buffer.
  char buf[50];
  snprintf(buf, sizeof(buf), "%s %04d-%02d-%02d %02d:%02d:%02d",
           day.c_str(),
           t.yr, t.mon, t.date,
           t.hr, t.min, t.sec);

  // Print the formatted string to serial so we can see the time.
  Serial.println(buf);
  //myNextion.setComponentText("page0.t9", String(t.date));
  //myNextion.setComponentText("page0.t10", String(t.mon));
  //myNextion.setComponentText("page0.t11", String(t.yr));
  //myNextion.setComponentText("page0.t14", String(day.c_str()));
}

void setTime(String timeString)
{
  rtc.writeProtect(false);
  rtc.halt(false);
                    
  // Make a new time object to set the date and time.
  // Sunday, September 22, 2013 at 01:38:50.

  const char *str = timeString.c_str(); // assume this string is result read from serial
  int year, month, day, hour, minute, second, dayIndex;

  if (sscanf(str, "%d,%d,%d,%d,%d,%d,%d", &year, &month, &day, &hour, &minute, &second, &dayIndex) == 7) {
    // do something with r, g, b
    Time t(year, month, day, hour, minute, second, indexAsDay(dayIndex));
    rtc.time(t);
  }
  // Set the time and date on the chip.
}

void setPump()
{
  if (pump_state == true) 
  {
    Serial.write("ps trueping");
    digitalWrite(2, LOW);
  }
  else
  {
    Serial.write("ps falseping");
    digitalWrite(2, HIGH);
  }
}
