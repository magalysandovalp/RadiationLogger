//Solar Charger Power Logger
//Light sensors and Power measurements
//Magaly Sandoval
//Compology
//December, 2014.
//V1.1
 
  //Libraries required
  #include <Wire.h>
  #include <SD.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_INA219.h>
  #include <Adafruit_TSL2561_U.h>
  #include "RTClib.h"
  
/* Power IC */

  Adafruit_INA219 ina219;

/*   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc. 
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.
*/
   
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 1);
  Adafruit_TSL2561_Unified tsl2 = Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 2);
  Adafruit_TSL2561_Unified tsl3 = Adafruit_TSL2561_Unified(TSL2561_ADDR_LOW, 3);

  // how many milliseconds between grabbing data and logging it. 
  #define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)
  
  // how many milliseconds before writing the logged data permanently to disk
  // set it to the LOG_INTERVAL to write each time (safest)
  // set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
  // the last 10 reads if power is lost but it uses less power and is much faster!
  #define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
  uint32_t syncTime = 0; // time of last sync()
  
  #define ECHO_TO_SERIAL   1 // echo data to serial port
  #define WAIT_TO_START    0 // Wait for serial input in setup()
  
  // the digital pins that connect to the LEDs
  #define redLEDpin 2
  #define greenLEDpin 3
  
  #define BANDGAPREF 14            // special indicator that we want to measure the bandgap
  
  #define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
  #define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off
    
  RTC_DS1307 RTC; // define the Real Time Clock object
  
  // for the data logging shield, we use digital pin 10 for the SD cs line
  const int chipSelect = 10;
  
  // the logging file
  File logfile;
       
  void error(char *str)
  {
    Serial.print("error: ");
    Serial.println(str);
    
    // red LED indicates error
    digitalWrite(redLEDpin, HIGH);
  
    while(1);
  }
  
  
  void setup(void)
  {
    uint32_t currentFrequency;
    
    Serial.begin(9600);
    Serial.println();
    Wire.begin(); //Join the bus as a master
    
    // use debugging LEDs
    pinMode(redLEDpin, OUTPUT);
    pinMode(greenLEDpin, OUTPUT);

    
  #if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available());
  #endif //WAIT_TO_START
  
    // initialize the SD card
    Serial.print("Initializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(10, OUTPUT);
    
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      error("Card failed, or not present");
    }
    Serial.println("card initialized.");
    
    // create a new file
    char filename[] = "Logger00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }
    
    if (! logfile) {
      error("couldnt create file");
    }
    
    Serial.print("Logging to: ");
    Serial.println(filename);
  
    // connect to RTC
    Wire.begin();  
    if (!RTC.begin()) {
      logfile.println("RTC failed");
  #if ECHO_TO_SERIAL
      Serial.println("RTC failed");
  #endif  //ECHO_TO_SERIAL
    }
    
    logfile.println("MILIS,DATETIME,LUX1,LUX2,LUX3,CURRENT,LOAD_VOLTAGE,POWER");    
  #if ECHO_TO_SERIAL
    Serial.println("MILIS,DATETIME,LUX1,LUX2,LUX3,CURRENT,LOAD_VOLTAGE,POWER");
  #endif //ECHO_TO_SERIAL
   
    // If you want to set the aref to something other than 5v
    analogReference(EXTERNAL);
    
    /* Initialize the Light sensor */
    
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    if(!tsl2.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561_2 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
      if(!tsl3.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561_2 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /*INA219 INITIATION */
  ina219.begin();
  
  /* We're ready to go! */
  Serial.println(""); 
    
  }
  
  void loop(void)
  {
    DateTime now;
  
    // delay for the amount of time we want between readings
    delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
    
    digitalWrite(greenLEDpin, HIGH);
    
    // log milliseconds since starting
    uint32_t m = millis();
    logfile.print(m);           // milliseconds since start
    logfile.print(", ");    
  #if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(", ");  
  #endif
  
    // fetch the time
    now = RTC.now();
    // log time
    logfile.print('"');
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print('"');
    logfile.print(",");

    
   //Write Light Data
  //*******************************************************************************
    /* Get a new sensor event */ 
  sensors_event_t event;
  sensors_event_t event2;
  sensors_event_t event3;

  tsl.getEvent(&event);
  tsl2.getEvent(&event2);
  tsl3.getEvent(&event3);
  
  if (event.light)
  {
    Serial.println("");
    Serial.print(event.light); Serial.println(" lux");
    logfile.print(event.light);
    logfile.print(",");
 }
  else if (event.light == 0)
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.print(40000); Serial.println(" lux");
    logfile.print(event.light);
    logfile.print(",");
  }
  if (event2.light)
  {
    Serial.print(event2.light); Serial.println(" lux2");
    logfile.print(event2.light);
    logfile.print(",");
  }
  else if (event2.light == 0)
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.print(40000); Serial.println(" lux2");
    logfile.print(event2.light);
    logfile.print(",");
  }
   if (event3.light)
  {
    Serial.print(event3.light); Serial.println(" lux3");
    logfile.print(event3.light);
    logfile.print(",");
  }
  else if (event3.light == 0)
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.print(40000); Serial.println(" lux3");
    logfile.print(event3.light);
    logfile.print(",");
    
  }
 
 // delay(500);
 
   //Write Power Data
  //*******************************************************************************
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();  
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  power = loadvoltage*current_mA*0.001;
  
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power); Serial.println(" W");
  Serial.println("");
  
  logfile.print(current_mA);
  logfile.print(",");
  logfile.print(loadvoltage);
  logfile.print(",");
  logfile.print(power);
  logfile.print(",");
                                                   
  //New Line
    logfile.println();
  #if ECHO_TO_SERIAL
    Serial.println();
  #endif // ECHO_TO_SERIAL
  
    digitalWrite(greenLEDpin, LOW);
  
    // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
    // which uses a bunch of power and takes time
    if ((millis() - syncTime) < SYNC_INTERVAL) return;
    syncTime = millis();
    
    // blink LED to show we are syncing data to the card & updating FAT!
    digitalWrite(redLEDpin, HIGH);
    logfile.flush();
    digitalWrite(redLEDpin, LOW);
        
  }
  
 
   //************************************************************************/
  
  // ---------------------LIGHT OPERATIONS----------------------------
   
   void displaySensorDetails(void)
{
  sensor_t sensor;
  sensor_t sensor2;
  sensor_t sensor3;
  tsl.getSensor(&sensor);
  tsl2.getSensor(&sensor2);
  tsl3.getSensor(&sensor3);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor1:       "); Serial.println(sensor.name);
  Serial.print  ("Sensor2:       "); Serial.println(sensor2.name);  
  Serial.print  ("Sensor2:       "); Serial.println(sensor3.name);
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500); 
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  tsl.setGain(TSL2561_GAIN_1X);
  tsl2.setGain(TSL2561_GAIN_1X);
  tsl3.setGain(TSL2561_GAIN_1X);
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl2.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl3.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
}



