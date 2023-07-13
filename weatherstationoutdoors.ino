// THIS INO IS OLD AND IS ONLY KEPT FOR HISTORICAL PURPOSES 

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


#include "math.h"

OneWire oneWire(D2);
DallasTemperature sensors(&oneWire);  //start up the DS18 temp probes
Adafruit_BME680 bme; // start up the BME680 weather probe via I2C


#define SEALEVELPRESSURE_HPA (1013.25)
#define tempoffset -0.9F
#define TTS 1000 //Time till screen update in ms
#define TTU 20 //Time till update in TTS intervals

//TODO: MANY OF THESE DECLARATIONS ARE OLD AND UNUSED.  
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
    bme.begin();
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
    delay(5000); //Required to stabilize wifi
    Blynk.begin(auth, IPAddress(192,168,50,197), 8080);
    terminal.clear();
    terminal.println("********************************");
    terminal.println("STARTING OUTDOOR WEATHER STATION");
    terminal.println(Time.timeStr()); //print current time to Blynk terminal
    terminal.print("Default LED brightness: ");
    terminal.println(RGB.brightness());
    RGB.brightness(255);
    terminal.print("Adjusted LED brightness: ");
    terminal.println(RGB.brightness());
    terminal.println("********************************");
    terminal.flush();
    Time.zone(-5);
    sensors.begin();
}

void loop() { //This is where all Arduinos store their "do this all the time" code
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
        bme.performReading();
        sensors.requestTemperatures();
        tempBME = bme.temperature;
        humBME = bme.humidity;
        gasBME = (1 / (bme.gas_resistance / 1000.0)) * 10;
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
        Blynk.virtualWrite(V1, (bme.pressure / 100.0));
        Blynk.virtualWrite(V2, dewpoint);
        Blynk.virtualWrite(V3, bme.humidity);
        Blynk.virtualWrite(V4, abshumBME);
        Blynk.virtualWrite(V5, sensors.getTempCByIndex(1));
        Blynk.virtualWrite(V6, bme.temperature);
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
        terminal.print("Last update: ");
        terminal.print(Time.timeStr()); //print current time to Blynk terminal
        terminal.println("");
        terminal.flush();
        old1p0 = new1p0; //reset data spike check variable
        old2p5 = new2p5;
        old10 = new10;
        firstvalue = 0; //we have run once now, no longer first run
    }
}


