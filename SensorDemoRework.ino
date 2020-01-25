// 433Mhz receiver configuration
static const unsigned int REC_RX_PIN = 2; // data input pin of the 433Mhz receiver

// FT007TH sensors configuration
static const unsigned int NUM_SENSORS       =    8; // 8 sensors/channels max
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

boolean noErrors = true; //flags if signal does not follow Manchester conventions

// Banks for multiple packets if required (at least one will be needed)
byte manchester[20]; //Stores 4 banks of manchester pattern decoded on the fly

// Collected Sensor data
float sensorTemp[NUM_SENSORS] = {};
float sensorHum[NUM_SENSORS] = {};
float sensorBat[NUM_SENSORS] = {}; // TODO


void setup() {
  Serial.begin(115200);
  pinMode(REC_RX_PIN, INPUT);

  invalidateManchester();
}


// Main routines, find header, then get in sync with it, get a packet and decode data in it. Abort in case of errors.
void loop() {
  if (!noErrors || (numBytes >= MAX_BYTES)) {
    init_globals();
  }

  while (digitalRead(REC_RX_PIN) != tempBit) {
    // Loop until a transition is found
  }

  // At data transition, half way through bit pattern, this should be where REC_RX_PIN==tempBit
  delayMicroseconds(SHORT_DELAY); // Skip ahead to 3/4 of the bit pattern

  // 3/4 the way through, if REC_RX_PIN has changed it is definitely an error
  if (digitalRead(REC_RX_PIN) != tempBit) {
    // Something has gone wrong, POLARITY has changed too early, ie always an error
    // Exit and retry
    noErrors = false;
    return;
  }

  delayMicroseconds(LONG_DELAY); // Skip ahead 1 quarter into the next bit pattern

  if (digitalRead(REC_RX_PIN) == tempBit) { // If REC_RX_PIN has not swapped, then bitWaveform is swapping
    //If the header is done, then it means data change is occuring ie 1->0, or 0->1
    //data transition detection must swap, so it loops for the opposite transitions
    tempBit = tempBit ^ 1;
  } //end of detecting no transition at end of bit waveform, ie end of previous bit waveform same as start of next bitwaveform

  // Now process the tempBit state and make data definite 0 or 1's, allow possibility of Pos or Neg POLARITY
  byte bitState = tempBit ^ POLARITY; //if POLARITY=1, invert the tempBit or if POLARITY=0, leave it alone.

  if (bitState == 1) { //1 data could be header or packet
    if (!firstZero) {
      headerHits++;
      if (headerHits == NUM_HEADER_BITS) {
        //valid header accepted, minimum required found
        //Serial.print("H");
      }
    } else {
      add(bitState); //already seen first zero so add bit in
    }
  } else { //bitState==0 could first error, first zero or packet
    // if it is header there must be no "zeroes" or errors

    if (headerHits < NUM_HEADER_BITS) {
      // Still in header checking phase, more header hits required
      noErrors = false; // Landing here means header is corrupted, so it is probably an error
      return;
    }
    
    // We have our header, chewed up any excess and here is a zero

    if (!firstZero) { //if first zero, it has not been found previously
      firstZero = true;
      add(bitState); //Add zero to bytes
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
    int stnId = ((manchester[3] & B01110000) / 16) + 1; // looks at 3 bits in byte 3 used to identify channels 1 to 8
    int dataType = manchester[1]; // looks in byte 1 for the F007th Ambient Thermo-Hygrometer code (0x45)
    int newTemp = (float((((manchester[3] & B00000111) * 256) + manchester[4]) - 720) * 0.0556); // looks in bytes 3 and 4 for temperature and then converts to C
    int newHum = (manchester[5]); // looks in byte 5 for humidity data

    if (dataType == 0x45) {
      if ((stnId > 0) && (stnId <= NUM_SENSORS)) {
        sensorTemp[stnId - 1] = newTemp;
        sensorHum[stnId - 1] = newHum;
      }
    }

    Serial.println("******************************");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print("Sensor Channel ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(sensorTemp[i], 1);
      Serial.print("°C ");
      Serial.print(sensorHum[i], 0);
      Serial.println("%");
    }
    Serial.println("******************************");
  }
}

void init_globals()
{
  tempBit = 1;
  noErrors = true;
  firstZero = false;
  headerHits = 0;
  numBits = 0;
  numBytes = 0;
}

void invalidateManchester() {
  // Fill the memory with non matching numbers across the banks
  // If there is only one packet, with no repeats this is not necessary
  for (int i = 0; i < 20; i++) {
    manchester[i] = i;
  }
}
