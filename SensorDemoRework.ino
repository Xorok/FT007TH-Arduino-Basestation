// 433Mhz receiver configuration
static const unsigned int REC_RX_PIN = 2; // data input pin of the 433Mhz receiver

// FT007TH sensors configuration
static const unsigned int NUM_SENSORS       =    8; // 8 sensors/channels max

// Sensor constants
static const unsigned int SHORT_DELAY       =  242;
static const unsigned int LONG_DELAY        =  484;
static const unsigned int POLARITY          =    1;
static const unsigned int NUM_HEADER_BITS   =   10;
static const unsigned int MAX_BYTES         =    6;

// Sensor variables
byte      tempBit;
boolean   firstZero;
byte      headerHits;
byte      dataByte;
byte      numBits;
byte      numBytes;
byte      manchester[MAX_BYTES];

boolean errors = false; // flags if signal does not follow Manchester conventions

// Collected Sensor data
float sensorTemp[NUM_SENSORS] = {};
int   sensorHum[NUM_SENSORS] = {};
int   sensorBat[NUM_SENSORS] = {};


void setup() {
  Serial.begin(115200);
  pinMode(REC_RX_PIN, INPUT);
}


// Main routines, find header, then get in sync with it, get a packet and decode data in it. Abort in case of errors.
void loop() {
  if (errors || numBytes >= MAX_BYTES) {
    initVariables();
  }

  while (digitalRead(REC_RX_PIN) != tempBit) {
    // Loop until a transition is found
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
    }

    printData();
  }
}


void initVariables() {
  tempBit = 1;
  errors = false;
  firstZero = false;
  headerHits = 0;
  numBits = 0;
  numBytes = 0;
}

void printData() {
  Serial.println("******************************");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor Channel ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorTemp[i], 1);
    Serial.print("°C ");
    Serial.print(sensorHum[i]);
    Serial.print("% ");
    Serial.println(sensorBat[i]);
  }
  Serial.println("******************************");
}
