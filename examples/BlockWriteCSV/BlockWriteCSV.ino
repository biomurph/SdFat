/*
*
*    Based on the Raw Write in this example folder
*    An easy way to log ASCII sensor or whatever data to CSV file
*    Functions to write ints, floats, and strings.
*
*
*    Made by Biomurph in winter 2019
*    based on code by Biomurph for OpenBCI in 2014
*
*    Tested on Adafruit Feather M0
*/


#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define COMMA 0
#define NEW_LINE 1
#define BLOCK_200 200  // SAMPLE RATE AND DATA PACKET SIZE WILL EFFECT THE TIME THIS TAKES
#define BLOCK_400 400
#define BLOCK_800 800

const int cardSelect = 4;
const uint32_t BLOCK_COUNT = BLOCK_400;
const uint32_t MICROS_PER_BLOCK = 2000; // minimum time that won't trigger overrun error
SdFat sd;  // file system
SdFile file;  // test file
uint32_t bgnBlock, endBlock; // file extent
uint16_t overruns = 0;  // how many times it took longer than MICROS_PER_BLOCK to write pCache
uint32_t maxWriteTime = 0;  // longest write time, seeded
uint16_t minWriteTime = 999999; // shortest write time, seeded
uint32_t runTime; // amount of time that it actually took to write BLOCK_COUNT
uint32_t blockCounter = 0;  // keeping track of written blocks
uint8_t* pCache;  // the buffer
volatile int16_t byteCounter = 0;     // used to hold position in pCache
// log of overruns
#define OVER_DIM 20
struct {
  uint32_t overBlock;   // holds block number that over-wrote
  uint32_t overMicros;  // holds the length of this of over-write
} over[OVER_DIM];

int ADCcounts;
float convertedCounts;

unsigned long toggleTimer;
int toggleTime = 500; // blink pulse width
boolean logData = false;

void setup(){
  Serial.begin(230400); // agree to talk fast
  toggleTime = 100;
  while (!Serial) {
     toggleLED(); // wait for serial port to connect. Needed for native USB port only
  }
    toggleTime = 600;
    initializeSD();  // establish SD card connection
    makeLogFile();   // name a file and set it up to write blocks
    logData = true;
  }

  void loop(){
    toggleLED();
    ADCcounts = analogRead(0);
    convertedCounts = 3.14 * 2 * float(ADCcounts);
    if(logData){
      SDlogDataPacket();
    }
    delay(10);  // approximate 100Hz
  }


// see if the SD card is present and can be initialized:
  void initializeSD() {
    if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
      Serial.println("SD fail");
    } else {
      Serial.println("SD success");
    }
  }

// name a new file, allocate space on the SD and open the file for writing
  void makeLogFile() {
    char filename[15];
    strcpy(filename, "/DATA-000.csv");
    for (uint8_t i = 0; i < 1000; i++) {
      filename[6] = '0' + i / 100;
      filename[7] = '0' + i / 10;
      filename[8] = '0' + i % 10;
      // create if does not exist, do not open existing
      if (! sd.exists(filename)) {
        break;
      }
    }
    // create a contiguous file
    if (!file.createContiguous(sd.vwd(), filename, 512UL * BLOCK_COUNT)) {
      Serial.println("createContiguous failed");
    }
    // get the location of the file's blocks
    if (!file.contiguousRange(&bgnBlock, &endBlock)) {
      Serial.println("contiguousRange failed");
    }
    // clear the cache and use it as a 512 byte buffer
    pCache = (uint8_t*)sd.vol()->cacheClear();
    // tell card to setup for multiple block write with pre-erase
    if (!sd.card()->erase(bgnBlock, endBlock)) {
      Serial.println("card.erase failed");
    }
    if (!sd.card()->writeStart(bgnBlock, BLOCK_COUNT)) {
      Serial.println("writeStart failed");
    }
    //initialize overrun variables
    overruns = 0;  // number of overruns
    maxWriteTime = 0;  // longest block write time
    minWriteTime = 9999999;  // shortest block write time
    blockCounter = 0;   // written block counter
    byteCounter = 0;    // counts up to 512

    logStrings("%,"); logStrings(filename); logStrings("\n");
    logStrings("%,Some Data Sample\n");
    logStrings("%,Sample Rate 80Hz\n");
    logStrings("%,Data Packet Structure\n");
    logStrings("%,Time stamp,Raw Counts,Converted Counts\n");

    //   <VERBOSE>
    Serial.print("Ready to write to "); Serial.println(filename);
    Serial.print("Start raw write of "); Serial.print(BLOCK_COUNT); Serial.print(" blocks, "); Serial.print(file.fileSize()); Serial.println(" bytes");
    Serial.print("Please wait ~");
  }

  void SDlogDataPacket() {
    logStrings(",");  // this is here to make the data columns line up with the header
    itos(micros(), COMMA); // machine time stamp, good for jitter detection, not much else
    itos(ADCcounts, COMMA); // int data
    ftos(convertedCounts,2,NEW_LINE); // float data
  }

  void closeLogFile() {
    pCache = (uint8_t*)sd.vol()->cacheClear();  // clear cache from last log. works??
    logMetaStats(); // put a footer on the CSV file to log duration and any overruns
    // end multiple block write mode
    if (!sd.card()->writeStop()) Serial.println("writeStop failed");
    file.close();
    logData = false;
  }



  //  count the int ascii bytes as they are buffered into pCache
  void itos(int intData, int format) {
    String stringData;
    stringData = String(intData);
    for (char c = 0; c < stringData.length(); c++) {
      pCache[byteCounter] = stringData.charAt(c);
      byteCounter++;
      if (byteCounter == 512) {
        writeCache();
      }
    }
    switch (format) {
      case COMMA:
        pCache[byteCounter] = ',';
        break;
      case NEW_LINE:
        pCache[byteCounter] = '\n';
        break;
      default:
        break;
    }
    byteCounter++;
    if (byteCounter == 512) {
      writeCache();
    }
  }

  //  count the float ascii bytes as they are buffered into pCache

  void ftos(float floatData, int decimalPlaces, int format) {
    String stringData;
    stringData = String(floatData, decimalPlaces);
    for (char c = 0; c < stringData.length(); c++) {
      pCache[byteCounter] = stringData.charAt(c);
      byteCounter++;
      if (byteCounter == 512) {
        writeCache();
      }
    }
    switch (format) {
      case COMMA:
        pCache[byteCounter] = ',';
        break;
      case NEW_LINE:
        pCache[byteCounter] = '\n';
        break;
      default:
        break;
    }
    byteCounter++;
    if (byteCounter == 512) {
      writeCache();
    }
  }

  void writeCache() {
   uint32_t writeTime = micros();  // time the write
    if (!sd.card()->writeData(pCache)) {
      Serial.println("writeData failed");
    }
   writeTime = micros() - writeTime;      // benchmark the write
   if (writeTime > maxWriteTime) {  // check for max write time
     maxWriteTime = writeTime;
   }
   if (writeTime < minWriteTime) {  // check for min write time
     minWriteTime = writeTime;
   }
   //     check for overrun
   if (writeTime > MICROS_PER_BLOCK) {
     if (overruns < OVER_DIM) {
       over[overruns].overBlock = blockCounter;
       over[overruns].overMicros = writeTime;
     }
     overruns++;
   }
    byteCounter = 0; // reset 512 byte counter
    blockCounter++;    // increment BLOCK counter
    if (blockCounter == BLOCK_COUNT - 1) {
      runTime = micros() - runTime;
      printSDstats();
      closeLogFile();
    }

  }

  void logStrings(String s) {
    // send strings to the SDcard
    for (char c = 0; c < s.length(); c++) {
      pCache[byteCounter] = s.charAt(c);
      byteCounter++;
      if (byteCounter == 512) {
        writeCache();
      }
    }
  }

  void logMetaStats() { // this takes up about 100 bytes
    logStrings("\nDone\n");
    logStrings("Elapsed time,") ; ftos( 1.e-6 * runTime, 2, COMMA); logStrings("seconds\n");
    logStrings("Max write time,"); itos(maxWriteTime, COMMA); logStrings("micros\n");
    logStrings("Min write time,"); itos(minWriteTime, COMMA); logStrings("micros\n");
    logStrings("Overruns,"); itos(overruns, NEW_LINE);
    if (overruns) {
      uint8_t n = overruns > OVER_DIM ? OVER_DIM : overruns;
      logStrings("fileBlock,micros\n");
      for (uint8_t i = 0; i < n; i++) {
        itos(over[i].overBlock, COMMA); itos(over[i].overMicros, NEW_LINE);
      }
    }
    writeCache();  // dump the last block
  }


  void printSDstats(){
      Serial.println("Done");
      Serial.print("Elapsed time: ") ; Serial.print( 1.e-6 * runTime); Serial.println(" seconds");
      Serial.print("Max write time: "); Serial.print(maxWriteTime ); Serial.println(" micros");
      Serial.print("Min write time: "); Serial.print(minWriteTime); Serial.println(" micros");
      Serial.print("Overruns: "); Serial.println(overruns);
      if (overruns) {
        uint8_t n = overruns > OVER_DIM ? OVER_DIM : overruns;
        Serial.println("fileBlock,micros");
        for (uint8_t i = 0; i < n; i++) {
          Serial.print(over[i].overBlock); Serial.print(','); Serial.println(over[i].overMicros);
        }
      }
  }

  void toggleLED(){
    if(millis() - toggleTimer > toggleTime){
      toggleTimer = millis();
      digitalWrite(13, !digitalRead(13));
    }
  }
