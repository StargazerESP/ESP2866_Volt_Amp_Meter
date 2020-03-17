#include <Wire.h> //ic2 interface
#include "heltec.h" // Manufacture Interface
#include <ESP8266WiFi.h> // Wifi connection

#include "Adafruit_ADS1015.h"// 16Bit I2C ADC+PGA ASD1115
#include "local_config.h" //My Wifi SSID, password, and MQTT Server IP

#include <PubSubClient.h>
#include <ArduinoJson.h>




/************ Global State ******************/
// System Values
String title_plate = "Stargazer ESP";
String device_plate = "Volt amp Sensor Ver 0.2";


// Network Values
byte wifi_mac[6];
String wifi_mac_str;
String wifi_mac_str_h; //Human Readable
IPAddress ip; 
String ip_str;
String ssid;

//Default Voltage Divider Values
int default_r_1 = 150000;
int default_r_2 = 15000;

//Default Shunt Value
int default_shunt_mva = 75;

//Reading Values
float volts, amps, watts = 0.00;


Adafruit_ADS1115 ads1115_0(0x48), ads1115_1(0x49), ads1115_2(0x4A), ads1115_3(0x4B);
byte ads_active[4];
float device_1_v, device_1_a, device_1_w;
float device_2_v, device_2_a, device_2_w;
float device_3_v, device_3_a, device_3_w;
float device_4_v, device_4_a, device_4_w;

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient espClient;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure espClient;

//MQTT Values
PubSubClient client(espClient);
const char* mqtt_server = MQTT_SERVER; // - MQTT_SERVER stored in local_config.h



/********** Functions **********/
void display_setup(void)
{
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Serial Enable*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  Heltec.display->display();  
  delay(100);
  Heltec.display->drawString(0,0,title_plate);
  Heltec.display->drawString(0,9,device_plate);
  Heltec.display->display();
  delay(3000);
  
}

void wifi_setup(void)
{
  Serial.println();
  Serial.println("Starting Wifi");
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Wifi Connecting...");
  Heltec.display->display();
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.macAddress(wifi_mac);
    String wifi_mac_byte_1 = String(wifi_mac[5],HEX);
    String wifi_mac_byte_2 = String(wifi_mac[4],HEX);
    String wifi_mac_byte_3 = String(wifi_mac[2],HEX);
    String wifi_mac_byte_4 = String(wifi_mac[1],HEX);
    String wifi_mac_byte_5 = String(wifi_mac[0],HEX);
    wifi_mac_str = wifi_mac_byte_1 + wifi_mac_byte_2 + wifi_mac_byte_3 + wifi_mac_byte_4 + wifi_mac_byte_5;
    wifi_mac_str_h = wifi_mac_byte_1 + ":" + wifi_mac_byte_2 + ":" + wifi_mac_byte_3 + ":" + wifi_mac_byte_4 + ":" + wifi_mac_byte_5;

    
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("Mac address: "); Serial.println(wifi_mac_str_h);
    Heltec.display->drawString(0, 9, "Wifi Connected");
    Heltec.display->display();
    delay(1000);
  }
  else
  {
    Heltec.display->clear();
    Heltec.display->drawString(0, 9, "Wifi Failed");
    Heltec.display->display();
    delay(1000);
  }
}

void ic2_setup(void)
{

  
  Wire.begin();// Use D2(16)=SDA, U1(24)=SCL 
  Serial.println();
  Serial.println("I2C Starting");
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "IC2 Connecting...");
  Heltec.display->display();
  delay(1000);  

  int scan_ids[]={60, 72, 73, 74, 75}; // The ID's for the OLED, and 4 ADS1115 address
  int scan_id_count = 5;
   
  Serial.println("Scanning...");

  byte error, address;
  int nDevices;
  String ic2_addr_str, ic2_device_name; 

  nDevices = 0;
  for(int device_scan_count =0; device_scan_count < scan_id_count; device_scan_count++)
  {
    address = scan_ids[device_scan_count];
    Serial.print("\n Address: ");
    Serial.println(address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("   I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      ic2_addr_str = String(address,HEX);
      String display_output_1 = "IC2 Device Found: ";
      String display_output_2 = "0x";
      display_output_2 += ic2_addr_str;
      
      // Device List base on expected Address
      switch (address) {
        case 60:
          // esp8622 Wifi Kit 8 - OLED Display
          ic2_device_name = "OLED Display";
          break;
        case 72:
          // Adafruit ADS1115 - 16bit X 4 DAC, Address 1
          ic2_device_name = "ADS1115 - 0";
          ads_active[0] = 1;
          break;
        case 73:
          // Adafruit ADS1115 - 16bit X 4 DAC, Address 2
          ic2_device_name = "ADS1115 - 1";
          ads_active[1] = 1;
          break;
        case 74:
          // Adafruit ADS1115 - 16bit X 4 DAC, Address 3
          ic2_device_name = "ADS1115 - 2";
          ads_active[2] = 1;
          break;
        case 75:
          // Adafruit ADS1115 - 16bit X 4 DAC, Address 4
          ic2_device_name = "ADS1115 - 3";
          ads_active[3] = 1;
          break;
        default:
          ic2_device_name += "Unknown";
      }
      
      String display_output_3 = "";
      display_output_3 += ic2_device_name;
      
      Serial.print("   ");
      Serial.println(ic2_device_name);
      Heltec.display->clear();
      Heltec.display->drawString(0, 0, display_output_1);
      Heltec.display->drawString(0, 9, display_output_2);
      Heltec.display->drawString(0, 18, display_output_3);
      Heltec.display->display();
      delay(3000);
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    } 
  }
      
  if (nDevices == 0)
   Serial.println("No I2C devices found\n");
  else
   Serial.println("done\n"); 
   
}

void display_device_info(void){
  Heltec.display->clear();
  Heltec.display->drawString(0,0,title_plate);
  Heltec.display->drawString(0,9,device_plate);
  Heltec.display->display();
}

void display_wifi(void){
  ip = WiFi.localIP();
  ip_str = WiFi.localIP().toString().c_str();
  ssid = WiFi.SSID();
  String display_output_1_1 = "SSID: ";
  display_output_1_1 += ssid;
  String display_output_1_2 = "Mac: ";
  display_output_1_2 += wifi_mac_str;
  String display_output_1_3 = "IP: ";
  display_output_1_3 += ip_str;

  Heltec.display->clear();
  Heltec.display->drawString(0, 0, display_output_1_1);
  Heltec.display->drawString(0, 9, display_output_1_2);
  Heltec.display->drawString(0, 18, display_output_1_3);
  Heltec.display->display();
}

float Ain_to_V(int16_t adc_in){
  // Convert the Analog in to Volts 
  float volts;
  volts = (adc_in * 0.1875) / 1000;
  return volts;
}

float Voltage_Divider(int16_t adc_in, int r1, int r2) {
  float v_divider;
  float volts;
  float real_volts;
  
  v_divider = r1 / r2;
  volts = Ain_to_V(adc_in);
  real_volts = volts * v_divider;
  return real_volts;
}

float ACS712_Ain_to_A(int ain){
  // ACS712ELC-30A currnet sensor
  int zero_ain_value;
  int act_ain_value;
  float v_out;
  float amps;

  zero_ain_value = 13604;
  act_ain_value = ain - zero_ain_value;
  v_out = Ain_to_V(act_ain_value);
  amps = v_out / 100;
  return amps;
}

void start_ads1115(void){
  if (ads_active[0] == 1) 
    ads1115_setup(0);
  if (ads_active[1] == 1)
  ads1115_setup(1);
  if (ads_active[2] == 1)
  ads1115_setup(2);
  if (ads_active[3] == 1)
  ads1115_setup(3);
}


void ads1115_setup(int ads_id)
{

    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "ADS1115 Connecting...");
    Heltec.display->display();
    delay(1000);
    Heltec.display->clear();

  //to do add address selection
  Serial.print("ADS1115 ID ");Serial.println(ads_id);
  switch (ads_id){
    case 0:
      Serial.println("Starting");
      ads1115_0.begin(); // Initialize ads1115
      Serial.println("ADC Range: +/- 6.144V (1 bit = 188uV/bit)");
      Serial.println();
    break;
    case 1:
      Serial.println("Starting");
      ads1115_1.begin(); // Initialize ads1115
      Serial.println("ADC Range: +/- 6.144V (1 bit = 188uV/bit)");
      Serial.println();
    break;
    case 2:
      Serial.println("Starting");
      ads1115_2.begin(); // Initialize ads1115
      Serial.println("ADC Range: +/- 6.144V (1 bit = 188uV/bit)");
      Serial.println();
    break;
    case 3:
      Serial.println("Starting");
      ads1115_3.begin(); // Initialize ads1115
      Serial.println("ADC Range: +/- 6.144V (1 bit = 188uV/bit)");
      Serial.println();
    break;
    default:
      Serial.println("Invalid ads_id");
      return;
  }
}

void read_device(int dev_id,int r_1 = default_r_1 ,int r_2 = default_r_2, int shunt_mva = default_shunt_mva){
  Serial.print("Device ID: ");Serial.println(dev_id);
    int16_t raw_adc_value_volts;
    int16_t raw_adc_value_amps;
  
  if(ads_active[dev_id] == 1){
    // dev_status  = "Online";
    switch (dev_id){
      case 0:
        raw_adc_value_volts = ads1115_0.readADC_Differential_0_1();
        raw_adc_value_amps = ads1115_0.readADC_Differential_2_3();
      break;
    }
    // Get Voltage Data
    
    volts = Voltage_Divider(raw_adc_value_volts, r_1, r_2);

    //Get Amp Data
        
    String dev_status_display = "unknown";
    String display_output_line_1 = "Meter ID = ";
    display_output_line_1 += dev_id;
          
    String display_output_line_2 = "Volts: ";
    display_output_line_2 += volts;
    display_output_line_2 += " Amps: ";
    display_output_line_2 += "aa.aa";
    
    String display_output_line_3 = "Watts: ";
    display_output_line_3 += " www.ww";
    Serial.print(" Volts: ");Serial.println(volts);
    Serial.print(" Amps: ");Serial.println(amps,3);
    Serial.print(" Watts: ");Serial.println(watts);
    Serial.println(" ");
    
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, display_output_line_1);
    Heltec.display->drawString(0, 9, display_output_line_2);
    Heltec.display->drawString(0, 18, display_output_line_3);
    Heltec.display->display();

    post_to_mqtt(dev_id,volts,amps,watts);
    


    delay(500);    
  }else{
    Serial.println(" Status Offline: ");
    Serial.println();    
  }  
}

void post_to_mqtt(int dev_id, float volts, float amps, float watts){
  String mqtt_volts;
  String mqtt_amps;
  String mqtt_watts;
  String mqtt_json_output;
  String mqtt_path = MQTT_BASE_PATH;  // Found in local_config.h
  mqtt_path += "Multimeter/";
  mqtt_path += wifi_mac_str_h;
  mqtt_path += "/";  
  mqtt_path += dev_id;

  // Connect to MQTT server
  if (!client.connected()) {
    reconnect();
  }
  StaticJsonDocument<1024> mqtt_json;  
  //convert float to string
  char con_buffer[10];
  mqtt_volts = dtostrf(volts,4,2, con_buffer);
  mqtt_amps = 0.00;//dtostrf(amps,7,2, con_buffer);
  mqtt_watts = 0.00; //dtostrf(watts,9,1, con_buffer);
  mqtt_json["data"]["volts"] = mqtt_volts;
  mqtt_json["data"]["amps"] = mqtt_amps;
  mqtt_json["data"]["watts"] = mqtt_watts;

  serializeJson(mqtt_json, mqtt_json_output);
    

  Serial.println("MQTT Post:");
  Serial.println(mqtt_json_output);
  Serial.println();
  
  client.publish(mqtt_path.c_str(), mqtt_json_output.c_str());  
}

void setup(){
  Serial.begin(115200);
  delay(10);
  Serial.println();
  display_setup();
  wifi_setup();
  ic2_setup(); 
  start_ads1115();
  
  Serial.println();
  Heltec.display->clear();

  client.setServer(mqtt_server, 1883);


}

///// MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      // client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  // Main Loop
  display_device_info();
  delay(500);
  display_wifi();
  delay(500);
  read_device(0);
  read_device(1);
  read_device(2);
  read_device(3); 
}
