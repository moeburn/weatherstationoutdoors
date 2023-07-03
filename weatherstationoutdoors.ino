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

//#include "ILI9163.h" //Library for LCD
//#include "blynk.h"
#include "math.h"
//#include "SparkFunPhant.h"
//#include "Adafruit_Sensor.h"
//#include "Adafruit_BME280.h"

//#include <OneWire.h>
//#include <DS18.h>
//#include <DallasTemperature.h>
//DS18 DS18sensor(D2);
OneWire oneWire(D2);
DallasTemperature sensors(&oneWire);

Adafruit_BME680 bme; // I2C





//#define BME_SCK 13
//#define BME_MISO 12
//#define BME_MOSI 11
//#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)


#define SEALEVELPRESSURE_HPA (1013.25)

#define tempoffset -0.9F




#define TTS 1000 //Time till screen update in ms
#define TTU 20 //Time till update in TTS intervals
int timerBlynk = TTU;

float old1p0, old2p5, old10, new1p0, new2p5, new10;

//char auth[] = "8157361d2bf349e88d49b58c605aef00"; //BLYNK
char auth[] = "X_pnRUFOab29d3aNrScsKq1dryQYdTw7"; //BLYNK






//Phant phant(server, publicKey, privateKey); // Create a Phant object

//Adafruit_BME280 bme; // I2C

//ILI9163 tft(A2, D4, D6); // Initialize LCD

float abshumBME, tempBME, presBME, humBME;

int timer1 = TTU; // How often to send sensor updates to Ubidots
int screentime = (TTS / 1000);
int debugcounter = 105;
int firstvalue = 1;

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

float presold, presnew, tempold, tempnew, humold, humnew, humtemp = 0;
float deltapres, deltatemp, deltahum = 0;
float fwind, windsum, windavg = 0;
int windcount = 0;



unsigned long lastmillis = 0;
unsigned long millisBlynk = 0;
unsigned long deltamillis = 0;
unsigned long pagemillis = 0;
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

PMS7003Serial<USARTSerial> pms7003(Serial1, D6);

void setup() { //This is where all Arduinos store the on-bootup code
  //RGB.control(true); //Turn off Photon's pulsing blue LED

  //====THIS WAS LCD STUFF====
  /*tft.begin();
  tft.fill(rgb(0, 0, 0)); //Clear LCD
  tft.setTextColor(WHITE, BLACK);
  tft.setFont(2);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("LOADING...");
  tft.copy_buffer();*/


  /*Particle.subscribe("Wind", windHandler, MY_DEVICES);
  Particle.subscribe("AbsHumidity", abshumHandler, MY_DEVICES);
  Particle.subscribe("Pressure", presHandler, MY_DEVICES);
  Particle.subscribe("Temperature", tempHandler, MY_DEVICES);*/
  Serial.begin();
bme.begin();
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  delay(5000); //Required to stabilize wifi
    Blynk.begin(auth, IPAddress(192,168,50,197), 8080);
  Time.zone(-5);
  sensors.begin();
  //bme.begin(0x76);
  //bme.setSampling(Adafruit_BME280::MODE_FORCED,
              //  Adafruit_BME280::SAMPLING_X1, // temperature
            //    Adafruit_BME280::SAMPLING_X1, // pressure
            //    Adafruit_BME280::SAMPLING_X1, // humidity
            //    Adafruit_BME280::FILTER_OFF   );
  //====THIS WAS LCD STUFF====
  /*
  pinMode(D5, INPUT_PULLUP); //Button
  attachInterrupt(D5, btnpush, FALLING);*/
  pagemillis = millis();


}

void loop() { //This is where all Arduinos store their "do this all the time" code
//====THIS WAS LCD STUFF====
/*
  switch (page)
    {
      case 1:
        tft.setFont(4);
        page1();
        break;
      case 2:
        tft.setFont(4);
        page2();
        break;
      case 3:
        tft.setFont(4);
        page3();
        break;
      case 4:
        tft.setFont(4);
        page4();
        break;
    }
    */
    pms7003.Read();
  Blynk.run();
  if (millis() - millisBlynk >= 2000)
  {
    pagetimer -= 2;
    
    /*bme.takeForcedMeasurement();
    tempBME = (bme.readTemperature() + tempoffset);
    presBME = (bme.readPressure() / 100.0F);
    humBME = bme.readHumidity();
    
    millisBlynk = millis();*/
    //DS18sensor.read();
    timerBlynk -= 2;
      if (timerBlynk <= 0)
      {
        timerBlynk = TTU;
        bme.performReading();
        sensors.requestTemperatures();
        //timerPhant -= 1;
        tempBME = bme.temperature;
        humBME = bme.humidity;
        
        new1p0 = pms7003.GetData(pms7003.pm1_0);
        new2p5 = pms7003.GetData(pms7003.pm2_5);
        new10 = pms7003.GetData(pms7003.pm10);
        if (firstvalue == 0)
        {
            if (new1p0 > 200) {new1p0 = old1p0;}
            if (new2p5 > 200) {new2p5 = old2p5;}
            if (new10 > 200) {new10 = old10;}
            if (new1p0 - old1p0 > 50) {new1p0 = old1p0;}
            if (new2p5 - old2p5 > 50) {new2p5 = old2p5;}
            if (new10 - old10 > 50) {new10 = old10;}
        }

        Blynk.virtualWrite(V1, (bme.pressure / 100.0));
        Blynk.virtualWrite(V0, sensors.getTempCByIndex(0));
        Blynk.virtualWrite(V3, bme.humidity);
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME+243.5))) * humBME * 2.1674)/(273.15 + tempBME);
        Blynk.virtualWrite(V4, abshumBME);
        Blynk.virtualWrite(V5, sensors.getTempCByIndex(1));
        Blynk.virtualWrite(V6, bme.temperature);
        Blynk.virtualWrite(V7, (bme.gas_resistance / 1000.0));
        //postToPhant();
        /*if (timerPhant <= 0)
        {
          timerPhant = 3;
          postToPhant();
        }*/
      }
  }
//====THIS WAS LCD STUFF====
  /*if ((millis() - pagemillis >= 20000) && (pagetimer <= 0))
  {
    pagemillis = millis();
    pagetimer = 0;
    button_push();
  }

  if (page > maxpages) {page=1;}*/

  if (millis() - deltamillis >= 3600000) //3600000 for one hour
  {
    deltamillis = millis();
  /*  humold = humnew;
    tempold = tempnew;
    presold = presnew;
    presnew = fpres;
    tempnew = ftemp;
    humnew = fhum;
    deltapres = presnew - presold;
    deltatemp = tempnew - tempold;
    deltahum = humnew - humold;

    //windavg = windsum / windcount;
    //windsum = 0;
    //windcount = 0;

    strcpy (deltatime, (Time.format(Time.now(), "%l:%M:%S %p")));*/
  }



}


