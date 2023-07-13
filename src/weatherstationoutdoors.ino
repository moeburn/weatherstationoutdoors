// This #include statement was automatically added by the Particle IDE.
#include <PMS7003-Particle-Sensor-Serial.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_BME680.h>

// This #include statement was automatically added by the Particle IDE.
#include <spark-dallas-temperature.h>

// This #include statement was automatically added by the Particle IDE.
#include <OneWire.h>

// This #include statement was automatically added by the Particle IDE.
#include <blynk.h>

#include <bsec.h>



#include "math.h"

const uint8_t bsec_config_iaq[] = {2,9,4,1,61,0,0,0,0,0,0,0,182,1,0,0,52,0,1,0,0,192,168,71,64,49,119,76,0,0,97,69,0,0,97,69,137,65,0,191,205,204,204,190,0,0,64,191,225,122,148,190,10,0,3,0,216,85,0,100,0,0,96,64,23,183,209,56,28,0,2,0,0,244,1,150,0,50,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,13,0,5,0,0,0,0,0,1,35,41,29,86,88,0,9,0,229,208,34,62,0,0,0,0,0,0,0,0,218,27,156,62,225,11,67,64,0,0,160,64,0,0,0,0,0,0,0,0,94,75,72,189,93,254,159,64,66,62,160,191,0,0,0,0,0,0,0,0,33,31,180,190,138,176,97,64,65,241,99,190,0,0,0,0,0,0,0,0,167,121,71,61,165,189,41,192,184,30,189,64,12,0,10,0,0,0,0,0,0,0,0,0,229,0,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,159,253,0,0};


OneWire oneWire(D2);
DallasTemperature sensors(&oneWire);  //start up the DS18 temp probes
Adafruit_BME680 bme; // start up the BME680 weather probe via I2C

#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000)

#define SEALEVELPRESSURE_HPA (1013.25)
#define tempoffset -0.9F
#define TTS 1000 //Time till screen update in ms
#define TTU 20 //Time till update in TTS intervals

//TODO: MANY OF THESE DECLARATIONS ARE OLD AND UNUSED.  

void checkIaqSensorStatus(void);
void errLeds(void);
void updateState(void);
void loadState(void);
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;
String output;

int timerBlynk = TTU;
float old1p0, old2p5, old10, new1p0, new2p5, new10;


char auth[] = "X_pnRUFOab29d3aNrScsKq1dryQYdTw7"; //auth token for Blynk - this is a LOCAL token, can't be used without LAN access

float abshumBME, tempBME, presBME, humBME, ds18temp, gasBME;

int timer1 = TTU; // How often to send sensor updates to Ubidots
int screentime = (TTS / 1000);
int debugcounter = 105;
int firstvalue = 1;  //important, do not delete

int history = 2;
int historycount = history;
float sensoravg;
float ftemp, fpres, fhum = 0;

float tempmax = -100000;
float tempmin = 100000;

float sensor;
float drawHeightold = 64;
int scrmax = 0;
int scrmin = 0;
float xmax, xmid, xmin;
float labelheight;

int last_pushed = 0; // global variable

#define maxpages 4 //How many screen pages we want
int page = 1;
int newpage = 1;
int ranged = 0;
float dewpoint = 0;
float presold, presnew, tempold, tempnew, humold, humnew, humtemp = 0;
float humidex = 0;
float deltapres, deltatemp, deltahum = 0;
float fwind, windsum, windavg = 0;
int windcount = 0;

float bmeiaq, bmeiaqAccuracy, bmestaticIaq, bmeco2Equivalent, bmebreathVocEquivalent, bmestabStatus, bmerunInStatus, bmegasPercentage;

unsigned long lastmillis = 0;
unsigned long millisBlynk = 0;
unsigned long deltamillis = 0;
unsigned long rgbmillis = 0;
int pagetimer = 0;

int xPos = 0;
char temp1[64];
char pres1[64];
char hum1[64];
char abshum1[64];
char wind1[64];
char wind2[64];
char time1[64];
char deltatime[64];

int timerPhant = 3;

int zebraR, zebraG, zebraB, sliderValue;
int menuValue = 2;
float  pmR, pmG, pmB;
bool rgbON = true;

PMS7003Serial<USARTSerial> pms7003(Serial1, D6);  //start up the PMS laser particle counter

WidgetTerminal terminal(V19); //terminal widget

BLYNK_WRITE(V21)
{
   menuValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V18)
{
     zebraR = param[0].asInt();
     zebraG = param[1].asInt();
     zebraB = param[2].asInt();
}

BLYNK_WRITE(V19)
{terminal.flush();}


void setup() { //This is where all Arduinos store the on-bootup code
  //RGB.control(true); //Turn off Photon's pulsing blue LED
    Serial.begin();
    iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
    /*bme.begin();
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms*/
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    delay(5000); //Required to stabilize wifi
    Blynk.begin(auth, IPAddress(192,168,50,197), 8080);
    terminal.clear();
    terminal.println("********************************");
    terminal.println("STARTING OUTDOOR WEATHER STATION");
    terminal.println(output);
    terminal.println(Time.timeStr()); //print current time to Blynk terminal
    terminal.print("Default LED brightness: ");
    terminal.println(RGB.brightness());
    RGB.brightness(255);
    terminal.print("Adjusted LED brightness: ");
    terminal.println(RGB.brightness());
    terminal.print("BSEC Config: ");
    for(int i = 0; i < 462; i++)
{
  terminal.print(bsec_config_iaq[i]);
}
terminal.println("");
        checkIaqSensorStatus();
    bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };
  iaqSensor.setTemperatureOffset(1.0);
  checkIaqSensorStatus();
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
   loadState();
  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

    // Print the header
    output = "Timestamp [ms], IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, raw temp[°C], pressure [hPa], raw relative humidity [%], gas [Ohm], Stab Status, run in status, comp temp[°C], comp humidity [%], gas percentage";
    terminal.println(output);
    terminal.println("********************************");
    terminal.flush();


    Time.zone(-5);
    sensors.begin();
}

void loop() { //This is where all Arduinos store their "do this all the time" code
  unsigned long time_trigger = millis();
                    if (iaqSensor.run()) { // If new data is available
                
                output = String(time_trigger);
                output += ", " + String(iaqSensor.iaq);
                output += ", " + String(iaqSensor.iaqAccuracy);
                output += ", " + String(iaqSensor.staticIaq);
                output += ", " + String(iaqSensor.co2Equivalent);
                output += ", " + String(iaqSensor.breathVocEquivalent);
                output += ", " + String(iaqSensor.rawTemperature);
                output += ", " + String(iaqSensor.pressure);
                output += ", " + String(iaqSensor.rawHumidity);
                output += ", " + String(iaqSensor.gasResistance);
                output += ", " + String(iaqSensor.stabStatus);
                output += ", " + String(iaqSensor.runInStatus);
                output += ", " + String(iaqSensor.temperature);
                output += ", " + String(iaqSensor.humidity);
                output += ", " + String(iaqSensor.gasPercentage);
                updateState();
                
            } else {
                checkIaqSensorStatus();
            }

    pms7003.Read();
    Blynk.run();
    if (menuValue == 1) {RGB.control(false);}
    if (menuValue == 2) 
    {
        pmG = 55 - sliderValue;
        if (pmG < 0) {pmG = 0;}
        pmG *= (255.0/55.0);
        if (pmG > 255) {pmG = 255;}
        
        pmR = sliderValue;
        if (pmR < 0) {pmR = 0;}
        pmR *= (255.0/55.0);
        if (pmR > 255) {pmR = 255;}
        
        pmB = sliderValue - 100;
        if (pmB < 0) {pmB = 0;}
        pmB *= (255.0/55.0);
        if (pmB > 255) {pmB = 255;}
        
        RGB.control(true);
        
        if (rgbON == true) {RGB.color(pmR, pmG, pmB);}
        else {RGB.color(0, 0, 0);}
        if (sliderValue > 55)
            {
                if (millis() - rgbmillis >= 500)
                {
                    if (rgbON) {rgbON = false;}
                    else {rgbON = true;}
                    rgbmillis = millis();
                }
            }
        else rgbON = true;
    } 
    if (menuValue == 3) 
        {
        RGB.control(true);
        RGB.color(zebraR, zebraG, zebraB);
        }
    
    if ((millis() - millisBlynk >= 30000) || (firstvalue == 1)) //every 30 seconds OR on first boot
    {

        millisBlynk = millis();
        //bme.performReading();
        sensors.requestTemperatures();
        tempBME = iaqSensor.temperature;
        humBME = iaqSensor.humidity;
        gasBME = (1 / (iaqSensor.gasResistance / 1000.0)) * 10;
        bmeiaq = iaqSensor.iaq;
        bmeiaqAccuracy = iaqSensor.iaqAccuracy;
        bmestaticIaq = iaqSensor.staticIaq;
        bmeco2Equivalent = iaqSensor.co2Equivalent;
        bmebreathVocEquivalent = iaqSensor.breathVocEquivalent;
        bmestabStatus = iaqSensor.stabStatus;
        bmerunInStatus = iaqSensor.runInStatus;
        bmegasPercentage = iaqSensor.gasPercentage;

        ds18temp = sensors.getTempCByIndex(0);
        new1p0 = pms7003.GetData(pms7003.pm1_0);
        new2p5 = pms7003.GetData(pms7003.pm2_5);
        new10 = pms7003.GetData(pms7003.pm10);
        
        if (firstvalue == 0)  //do not do this on the first run
        {
            if (new1p0 > 200) {new1p0 = old1p0;} //check for data spikes in particle counter, ignore data that is >200
            if (new2p5 > 200) {new2p5 = old2p5;} //data spikes ruin pretty graph
            if (new10 > 200) {new10 = old10;}
            if (new1p0 - old1p0 > 50) {new1p0 = old1p0;} //also ignore data that is >50 off from last data
            if (new2p5 - old2p5 > 50) {new2p5 = old2p5;}
            if (new10 - old10 > 50) {new10 = old10;}
        }
        
        
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME); //calculate absolute humidity
        dewpoint = tempBME - ((100 - humBME)/5); //calculate dewpoint
        humidex = ds18temp + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10); //calculate humidex using Environment Canada formula
        sliderValue = new2p5;
        Blynk.virtualWrite(V0, ds18temp);
        Blynk.virtualWrite(V1, (iaqSensor.pressure / 100.0));
        Blynk.virtualWrite(V2, dewpoint);
        Blynk.virtualWrite(V3, humBME);
        Blynk.virtualWrite(V4, abshumBME);
        Blynk.virtualWrite(V5, sensors.getTempCByIndex(1));
        Blynk.virtualWrite(V6, tempBME);
        Blynk.virtualWrite(V7, gasBME);
        Blynk.virtualWrite(V8, new1p0);
        Blynk.virtualWrite(V9, new2p5);
        Blynk.virtualWrite(V10, new10);
        Blynk.virtualWrite(V11, pms7003.GetData(pms7003.count0_3um));
        Blynk.virtualWrite(V12, pms7003.GetData(pms7003.count0_5um));
        Blynk.virtualWrite(V13, pms7003.GetData(pms7003.count1um));
        Blynk.virtualWrite(V14, pms7003.GetData(pms7003.count2_5um));
        Blynk.virtualWrite(V15, pms7003.GetData(pms7003.count5um));
        Blynk.virtualWrite(V16, pms7003.GetData(pms7003.count10um));
        Blynk.virtualWrite(V17, humidex);
        Blynk.virtualWrite(V23, bmeiaq);
        Blynk.virtualWrite(V24, bmeiaqAccuracy);
        Blynk.virtualWrite(V25, bmestaticIaq);
        Blynk.virtualWrite(V26, bmeco2Equivalent);
        Blynk.virtualWrite(V27, bmebreathVocEquivalent);
        Blynk.virtualWrite(V28, bmestabStatus);
        Blynk.virtualWrite(V29, bmerunInStatus);
        Blynk.virtualWrite(V30, bmegasPercentage);

        //terminal.println(output);
        //terminal.print("Last update: ");
        //terminal.print(Time.timeStr()); //print current time to Blynk terminal
        //terminal.println("");
        //terminal.flush();
        old1p0 = new1p0; //reset data spike check variable
        old2p5 = new2p5;
        old10 = new10;
        firstvalue = 0; //we have run once now, no longer first run
    }
}


void checkIaqSensorStatus(void)
{
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      terminal.println(output);
      //for (;;)
        //errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      terminal.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      terminal.println(output);
      //for (;;)
        //errLeds(); /* Halt in case of failure */
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      terminal.println(output);
    }
  }
}

void errLeds(void)
{

}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    terminal.println("**Reading state from EEPROM");
            terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("**");
       
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      terminal.println(bsecState[i], HEX);
      terminal.flush();
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    terminal.println("**Erasing EEPROM");
        terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("**");
        terminal.flush();
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    //EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  if (stateUpdateCounter == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    terminal.println("**Writing state to EEPROM");
        terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("**");
        
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      terminal.println(bsecState[i], HEX);
      terminal.flush();
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    //EEPROM.commit();
  }
}

