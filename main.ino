

// Your GPRS credentials (leave empty, if not needed)
const char apn[]      = "internet.comcel.com.co"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password


// Informacion del servidor
const char server[] = "161.97.72.238";
const char resource[] = "/report/4.23123/-4.2233";
const char serverFull[] = "bike-ccamilo2303.cloud.okteto.net/report/5.55555/-9.99999";
const int  port = 8080;

//GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;
#define RX 2
#define TX 15
// *********************** 

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
// BME280 pins
#define I2C_SDA_2            18
#define I2C_SCL_2            19

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);
//TinyGsmClientSecure client(modem);

#define uS_TO_S_FACTOR 1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3600        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

int x = 0;

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

void setup() {

  
  SerialMon.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX, TX);

  
  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

}

void loop() {
   while (Serial2.available()) {
    if(gps.encode(Serial2.read())){
 if(gps.location.isValid()){
    Serial.print(gps.location.lat(), 8);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 8);

    SerialMon.print("Connecting to APN....: ");
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(" fail");
    }
    else {
      SerialMon.println(" OK");
      SerialMon.print("Connecting to ");
      SerialMon.println(server);
      if (!client.connect(server, port)) {
        SerialMon.println(" fail");
        //delay(10000);
        return;
      }
      SerialMon.println(" success");
      x = x + 1;
      // Make a HTTP GET request:
      SerialMon.println("Performing HTTP GET request...");
      client.print(String("GET ") + "/report/" +gps.location.lat() +"/"+gps.location.lng() + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.print("Connection: close\r\n\r\n");
      client.println();
    
      uint32_t timeout = millis();
      while (client.connected() && millis() - timeout < 10000L) {
        // Print available data
        while (client.available()) {
          char c = client.read();
          SerialMon.print(c);
          timeout = millis();
        }
      }
      SerialMon.println();
    
      // Shutdown
    
      client.stop();
      SerialMon.println(F("Server disconnected"));
    
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
  }

    
  }else{
    Serial.print(F("INVALID "));
  }
  Serial.println();
      
      
  
  esp_deep_sleep_start();
     }
   }
    
  
    
}
