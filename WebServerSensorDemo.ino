#include <Ethernet.h>
#include <SD.h>
#include <SPI.h>


// Configuration
// *************

// Web server
static byte MAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
static IPAddress IP(192, 168, 0, 45);
EthernetServer server(80);

// 433Mhz receiver
static const unsigned int REC_RX_PIN = 2; // data input pin of the 433Mhz receiver

// FT007TH sensors
static const unsigned int NUM_SENSORS = 8; // 8 sensors/channels max

// SD storage
static const unsigned long saveIntervalMs = 10 * 60 * 1000UL; // 10 minutes

// *************

static const unsigned int SHORT_DELAY       =  242;
static const unsigned int LONG_DELAY        =  484;
static const unsigned int POLARITY          =    1;
static const unsigned int NUM_HEADER_BITS   =   10;
static const unsigned int MAX_BYTES         =    6;

byte      tempBit;
boolean   firstZero;
byte      headerHits;
byte      dataByte;
byte      numBits;
byte      numBytes;
byte      manchester[MAX_BYTES];
boolean   errors;

unsigned long previousSaveMs = 0;

// Collected Sensor data
float sensorTemp[NUM_SENSORS] = {};
int   sensorHum[NUM_SENSORS] = {};
int   sensorBat[NUM_SENSORS] = {};


void setup() {
  Serial.begin(115200);

  // Disable SD SPI
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // Disable w5100 SPI
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  if (!SD.begin(4)) {
    Serial.println(F("ERROR - SD card initialization failed!"));
    return;
  }

  /*if (!SD.exists(F("index.htm"))) {
    Serial.println(F("ERROR - Can't find index.htm file!"));
    return;
    }*/

  Ethernet.begin(MAC, IP);
  server.begin(); // start to listen for clients

  // Initialize 433Mhz receiver
  pinMode(REC_RX_PIN, INPUT);

  initVariables();
}


void loop() {
  listenForEthernetClients();

  readSensorSignals();
}


void listenForEthernetClients() {
  EthernetClient client = server.available();

  if (client) {
    // an http request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html; charset=utf-8"));
          client.println();
          // print the current readings, in HTML format:
          for (int i = 0; i < NUM_SENSORS; i++) {
            client.print(F("Sensor "));
            client.print(i + 1);
            client.print(F(": "));
            client.print(sensorTemp[i]);
            client.print(F("&#xB0;C / "));
            client.print(sensorHum[i]);
            client.print(F("% / "));
            client.print(sensorBat[i]);
            client.println(F("<br>"));
          }
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
}


// Main routines, find header, then get in sync with it, get a packet and decode data in it. Abort in case of errors.
void readSensorSignals() {
  if (errors || numBytes >= MAX_BYTES) {
    initVariables();
  }

  if (digitalRead(REC_RX_PIN) != tempBit) {
    // Loop until a transition is found
    return;
  }

  delayMicroseconds(SHORT_DELAY); // Skip ahead to 3/4 of the bit pattern

  if (digitalRead(REC_RX_PIN) != tempBit) {
    // Something has gone wrong, POLARITY has changed too early, ie always an error
    // Exit and retry
    errors = true;
    return;
  }

  delayMicroseconds(LONG_DELAY); // Skip ahead 1 quarter into the next bit pattern

  if (digitalRead(REC_RX_PIN) == tempBit) {
    tempBit = tempBit ^ 1;
  }

  byte bitState = tempBit ^ POLARITY; //if POLARITY=1, invert the tempBit or if POLARITY=0, leave it alone.

  if (bitState == 1) {
    if (!firstZero) {
      headerHits++;
    } else {
      add(bitState);
    }
  } else {
    if (headerHits < NUM_HEADER_BITS) {
      errors = true; // Landing here means header is corrupted, so it is probably an error
      return;
    }

    if (!firstZero) {
      firstZero = true;
      add(bitState);
      dataByte = B11111111;
      numBits = 7;
    } else {
      add(bitState);
    }
  }
}


void add(byte bitData) {
  dataByte = (dataByte << 1) | bitData;
  numBits++;

  if (numBits == 8) {
    numBits = 0;
    manchester[numBytes] = dataByte;
    numBytes++;
  }

  if (numBytes == MAX_BYTES) {
    int ch = (manchester[3] & B01110000) / 16 + 1; // looks at 3 bits in byte 3 used to identify channels 1 to 8
    int dataType = manchester[1]; // looks in byte 1 for the FT007th Ambient Thermo-Hygrometer code (0x45)
    float newTemp = float((manchester[3] & B00000111) * 256 + manchester[4] - 720) * 0.0556; // looks in bytes 3 and 4 for temperature and then converts to C
    int newHum = manchester[5];
    int lowBat = manchester[3] & 0x80 / 128;

    if (dataType == 0x45) {
      if (ch > 0 && ch <= NUM_SENSORS) {
        sensorTemp[ch - 1] = newTemp;
        sensorHum[ch - 1] = newHum;
        sensorBat[ch - 1] = lowBat;
      }

      printData();

      unsigned long currentMs = millis();
      if (currentMs - previousSaveMs > saveIntervalMs) {
        previousSaveMs = currentMs;
        writeDataToSd(currentMs);
      }
    }
  }
}


void writeDataToSd(long currentMs) {
  String data = "";

  // Time,Channel,Temperature,Humidity,Battery
  for (int i = 0; i < NUM_SENSORS; i++) {
    data += String(currentMs) + ',' + String(i + 1) + ',' + String(sensorTemp[i]) + ',' + String(sensorHum[i]) + ',' + String(sensorBat[i]) + '\n';
  }

  File dataFile = SD.open(F("data.csv"), FILE_WRITE);

  if (dataFile) {
    dataFile.print(data);
    dataFile.close();
  } else {
    Serial.println(F("Error opening data.csv"));
  }
}


void printData() {
  Serial.println(F("******************************"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("Sensor Channel "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(sensorTemp[i], 1);
    Serial.print(F("°C "));
    Serial.print(sensorHum[i]);
    Serial.print(F("% "));
    Serial.println(sensorBat[i]);
  }
  Serial.println(F("******************************"));
}


void initVariables() {
  tempBit = 1;
  errors = false;
  firstZero = false;
  headerHits = 0;
  numBits = 0;
  numBytes = 0;
}
