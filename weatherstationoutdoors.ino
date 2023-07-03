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

/*/*
void tempHandler(const char *event, const char *data) //Particle.subscribe events
{
  //detachInterrupt(D5);
strcpy (temp1, data);
sensoravg += strtof (temp1, NULL);
ftemp = strtof (temp1, NULL);
strcpy (time1, (Time.format(Time.now(), "%l:%M:%S %p")));
//====THIS WAS LCD STUFF====
/*
  if (historycount >= history)
  {
    historycount = 0;
        if (firstvalue == 1) {sensor = sensoravg;} else {sensor = sensoravg/history;}
        sensoravg = 0;
        sensor *= 100;
        if (scrmax == 0) {scrmax = sensor + 20;}
        if (scrmin == 0) {scrmin = sensor - 20;}
        if (sensor > scrmax) { scrmax = sensor; ranged = 1; }
        if (sensor < scrmin) { scrmin = sensor; ranged = 1; }

      if (drawArray[1] == 0 && drawArray[69] == 0 && drawArray[100] == 0) //FIRST TIME FILL
      {
        for(int i(0); i < 105; ++i)
         {
           drawArray[i] = sensor;
         }
        firstvalue = 0;
      }

      drawArray[104] = sensor; //set newest to newest

      for(int i(0); i < 105; ++i) //shift over one
      {
         tempArray[i-1] = drawArray[i];
       }
      for(int i(0); i < 105; ++i)
      {
         drawArray[i] = tempArray[i];
       }

       if (ranged == 1) //auto ranging
       {
         tempmax = -100000;
         tempmin = 100000;
         for(int i = 2; i<103; i++)
         {
              if(drawArray[i] > tempmax) {tempmax = drawArray[i];}
         }
         scrmax = tempmax;
         for(int i = 2; i<103; i++)
         {
              if(drawArray[i] < tempmin) {tempmin = drawArray[i];}
         }
         scrmin = tempmin;
       }

       xmax = float(scrmax) / 100;
       xmin = float(scrmin) / 100;
       xmid = (xmax + xmin) / 2;

  }

  historycount++;
  //attachInterrupt(D5, btnpush, FALLING);
}


void humHandler(const char *event, const char *data)
{
    //detachInterrupt(D5);
strcpy (hum1, data);

//  attachInterrupt(D5, btnpush, FALLING);
}

void abshumHandler(const char *event, const char *data)
{
    //  detachInterrupt(D5);
strcpy (abshum1, data);
fhum = strtof (abshum1, NULL);
//  attachInterrupt(D5, btnpush, FALLING);
}

void presHandler(const char *event, const char *data)
{
      //detachInterrupt(D5);
strcpy (pres1, data);
fpres = strtof (pres1, NULL);
//  attachInterrupt(D5, btnpush, FALLING);
}

/*void button_push()
{
  newpage = 1;

  if (page < maxpages) {
    lastmillis = millis()+20000;
    //scrmax = 0, scrmin = 0;
    //xPos=0;
    page+=1;

  }
    else {page=1;}
    tft.fill(rgb(0, 0, 0)); //Clear the LCD
    tft.setTextColor(WHITE, BLACK);
    tft.setFont(2);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.print("LOADING PAGE #");
    tft.print(page);
    tft.copy_buffer();
}*/

/*BLYNK_WRITE(V0) { //BLYNK Button press
  if (param.asInt() == 1) {
     // button_push();
    }
}*/

/*void btnpush()
{
  if (millis() > (last_pushed + 100)) { button_push();   pagetimer = 300; last_pushed = millis(); }
}*/

/*void page1() //MAIN DISPLAY
{
  if (millis() - lastmillis >= TTS)
  { //Update every two seconds (TTS), this will be equal to reading frequency (Hz).
    detachInterrupt(D5);



    tft.fill(rgb(0, 0, 0)); //Clear the LCD

    lastmillis = millis(); // Update lastmillis

    tft.drawLine(0,25,128,25,GREY);
    tft.setTextSize(3); //Biggest text size my tiny LCD can use
    tft.setCursor(0, 30); //Start printing text at x,y of 0,0
    tft.setTextColor(RED, BLACK); //Temperature

    tft.print(temp1);

    tft.setCursor(100, 30);
    tft.println("C");

    tft.setCursor(0, 60); //Humidity
    tft.setTextColor(BLUE, BLACK);
    tft.setTextSize(2);
    tft.print(abshum1);
    tft.println(" g/m3 ");
    tft.setTextSize(1);
    tft.println("");

    tft.setTextColor(CYAN, BLACK);
    tft.print(wind1);
    tft.print(" to ");
    tft.print(wind2);
    tft.println(" kph");

    //tft.println("");
    //tft.print(windavg);
    //tft.println(" kph avg/hr");


    tft.setTextColor(GREEN, BLACK); //Pressure
    tft.println("");
    tft.print(pres1);
    tft.println(" mBar");



    tft.drawCircle(93,33,2,RED); //My font library is dumb so just draw the little degree character manually
    tft.drawCircle(93,33,3,RED); //Draw it twice with larger radius to make it 2px thick

    tft.drawLine(0,112,128,112,GREY);
    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 115);
    tft.print("Last update: ");
    tft.print(time1);





    tft.setTextColor(YELLOW, BLACK);
    tft.setTextSize(2); //Smallest font size available
    tft.setCursor(0, 0); //x,y position to print text, at the bottom
    tft.print(Time.format(Time.now(), "%l:%M:%S%p"));
    //tft.print(" OUTDOORS");

    tft.copy_buffer(); //Nothing actually gets drawn on screen until we run this code, until now we were just filling display buffer
  attachInterrupt(D5, btnpush, FALLING);

  }
}





void page2()
{


  if (millis() - lastmillis >= 2000)
  { //Update every two seconds (TTS), this will be equal to reading frequency (Hz).

    detachInterrupt(D5);



    tft.fill(rgb(0, 0, 0)); //Clear the LCD

    lastmillis = millis(); // Update lastmillis

    tft.drawLine(0,10,128,10,GREY);
    tft.setTextSize(3); //Biggest text size my tiny LCD can use
    tft.setCursor(0, 15); //Start printing text at x,y of 0,0
    tft.setTextColor(RED, BLACK); //Temperature

    tft.print(tempBME);

    tft.setCursor(100, 15);
    tft.println("C");

    tft.setCursor(0, 55); //Humidity
    tft.setTextColor(BLUE, BLACK);
    tft.setTextSize(2);
    tft.print(abshumBME);
    tft.println(" g/m3 ");
    tft.setTextSize(1);
    tft.println("");
    tft.setTextColor(CYAN, BLACK);
    tft.print(humBME);
    tft.println("%");


    tft.setTextColor(GREEN, BLACK); //Pressure
    tft.println("");
    tft.print(presBME);
    tft.println(" mBar");



    tft.drawCircle(93,18,2,RED); //My font library is dumb so just draw the little degree character manually
    tft.drawCircle(93,18,3,RED); //Draw it twice with larger radius to make it 2px thick


    tft.drawLine(0,112,128,112,GREY);
    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 115);
    tft.print("INDOORS ");

    //DEBUG
    //tft.print("Debug: ");
    //tft.print(pagedebug);



    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1); //Smallest font size available
    tft.setCursor(0, 0); //x,y position to print text, at the bottom
    tft.print(Time.format(Time.now(), "%l:%M:%S %p"));

    tft.copy_buffer(); //Nothing actually gets drawn on screen until we run this code, until now we were just filling display buffer
  attachInterrupt(D5, btnpush, FALLING);

  }

}


void page3()
{

  if (millis() - lastmillis >= 10000) {
    detachInterrupt(D5);
    if (newpage == 1) {
      tft.fill(rgb(0, 0, 0)); //Clear the LCD
      newpage = 0;
      sensor = strtof (temp1, NULL);
      sensor *= 100;
      if (scrmax == 0) {scrmax = sensor + 50;}
      if (scrmin == 0) {scrmin = sensor - 50;}
      if (sensor > scrmax) { scrmax = sensor; }
      if (sensor < scrmin) { scrmin = sensor; }
      xmax = float(scrmax) / 100;
      xmin = float(scrmin) / 100;
      xmid = (xmax + xmin) / 2;
    }

        lastmillis = millis(); // Update lastmillis
        tft.fill(rgb(0, 0, 0));
        tft.setTextSize(1); //Biggest text size my tiny LCD can use
        tft.setTextColor(GREY, BLACK); //Temperature

        tft.setCursor(105, 0);
        tft.print(xmax);
        tft.setCursor(105,60);
        tft.print(xmid);
        tft.setCursor(105,120);
        tft.print(xmin);

        tft.setTextColor(RED, BLACK);
        labelheight = 120-map(drawArray[103], scrmin, scrmax, 0, 120);
        if (labelheight < 0) {labelheight = 0;}
        if (labelheight > 120) {labelheight = 120;}
        tft.setCursor(105, labelheight); //Start printing text at x,y of 0,0
        tft.print(float(drawArray[103]/100.00F));
        tft.setCursor(25, 3);
        tft.print(time1);


        tft.drawLine(0,64,103,64,GREY);
        tft.drawLine(0,0,103,0,GREY);
        tft.drawLine(0,127,103,127,GREY);
        tft.drawLine(78,11,78+((history-historycount)*2),11,BROWN);
        tft.drawLine(78,12,78+((history-historycount)*2),12,BROWN);
        if (scrmin < 0 && scrmax > 0)
        {
          float zeroheight = map(0, scrmin, scrmax, 0, 127);
          tft.drawLine(0, 127-zeroheight, 103, 127-zeroheight, CYAN);
        }

    for (int i = 0; i<104; i++)
    {
        scaleddrawArray[i] = map(drawArray[i], scrmin, scrmax, 0, 127);

    }
    for (int i = 1; i<104; i++)
    {
        tft.drawLine (i-1, 127-scaleddrawArray[i-1], i, 127-scaleddrawArray[i], FULLRED );
    }
        tft.copy_buffer();
          attachInterrupt(D5, btnpush, FALLING);
  }
}

int postToPhant()
{
  phant.add("abshum", abshumBME);
  phant.add("hum", humBME);
  //phant.add("pres", presBME);
  phant.add("temp", tempBME);

  TCPClient client;
  char response[512];
  int i = 0;
  int retVal = 0;

  if (client.connect(server, 80)) // Connect to the server
  {
  // Post message to indicate connect success
      //Serial.println("Posting!");

  // phant.post() will return a string formatted as an HTTP POST.
  // It'll include all of the field/data values we added before.
  // Use client.print() to send that string to the server.
      client.print(phant.post());
      //delay(1000);
  // Now we'll do some simple checking to see what (if any) response
  // the server gives us.
      while (client.available())
      {
          char c = client.read();
          //Serial.print(c);	// Print the response for debugging help.
          if (i < 512)
              response[i++] = c; // Add character to response string
      }
  // Search the response string for "200 OK", if that's found the post
  // succeeded.
      if (strstr(response, "200 OK"))
      {
          //Serial.println("Post success!");
          retVal = 1;
      }
      else if (strstr(response, "400 Bad Request"))
      {	// "400 Bad Request" means the Phant POST was formatted incorrectly.
    // This most commonly ocurrs because a field is either missing,
    // duplicated, or misspelled.
          //Serial.println("Bad request");
          retVal = -1;
      }
      else
      {
    // Otherwise we got a response we weren't looking for.
          retVal = -2;
      }
  }
  else
  {	// If the connection failed, print a message:
      //Serial.println("connection failed");
      retVal = -3;
  }
  client.stop();	// Close the connection to server.
  return retVal;	// Return error (or success) code.
}

void page4()
{
  if (millis() - lastmillis >= 2000)
  { //Update every two seconds (TTS), this will be equal to reading frequency (Hz).

    detachInterrupt(D5);



    tft.fill(rgb(0, 0, 0)); //Clear the LCD

    lastmillis = millis(); // Update lastmillis

    tft.drawLine(0,10,128,10,GREY);
    tft.setTextSize(2); //Biggest text size my tiny LCD can use
    tft.setCursor(0, 15); //Start printing text at x,y of 0,0
    tft.setTextColor(RED, BLACK); //Temperature

    tft.print(deltatemp);

    //tft.setCursor(100, 15);
    tft.println(" C/h");

    tft.setCursor(0, 55); //Humidity
    tft.setTextColor(BLUE, BLACK);
    tft.setTextSize(2);
    tft.print(deltahum);
    tft.println(" g/h");


    tft.setTextColor(GREEN, BLACK); //Pressure
    tft.println("");
    tft.print(deltapres);
    tft.println(" mB/h");



    //tft.drawCircle(95,18,2,RED); //My font library is dumb so just draw the little degree character manually
    //tft.drawCircle(95,18,3,RED); //Draw it twice with larger radius to make it 2px thick


    tft.drawLine(0,112,128,112,GREY);
    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 115);
    tft.print("LAST UPDATE: ");
    tft.print(deltatime);

    //DEBUG
    //tft.print("Debug: ");
    //tft.print(pagedebug);



    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1); //Smallest font size available
    tft.setCursor(0, 0); //x,y position to print text, at the bottom
    tft.print("DELTA VALUES-");
    tft.print(Time.format(Time.now(), "%l:%M:%S %p"));

    tft.copy_buffer(); //Nothing actually gets drawn on screen until we run this code, until now we were just filling display buffer
  attachInterrupt(D5, btnpush, FALLING);

  }

}

void page5()
{
  if (millis() - lastmillis >= 2000)
  { //Update every two seconds (TTS), this will be equal to reading frequency (Hz).

    detachInterrupt(D5);



    tft.fill(rgb(0, 0, 0)); //Clear the LCD

    lastmillis = millis(); // Update lastmillis

    tft.drawLine(0,10,128,10,GREY);
    tft.setTextSize(1); //Biggest text size my tiny LCD can use
    tft.setCursor(0, 15); //Start printing text at x,y of 0,0

    tft.setTextColor(RED, BLACK); //Temperature
    tft.print(tempold);
    tft.print(", ");
    tft.println(tempnew);
    tft.println("");

    tft.setTextColor(BLUE, BLACK);
    tft.print(humold);
    tft.print(", ");
    tft.println(humnew);
    tft.println("");



    tft.setTextColor(GREEN, BLACK); //Pressure
    tft.print(presold);
    tft.print(", ");
    tft.print(presnew);





    tft.drawLine(0,112,128,112,GREY);
    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 115);
    tft.print("LAST UPDATE: ");
    tft.print(deltatime);

    //DEBUG
    //tft.print("Debug: ");
    //tft.print(pagedebug);



    tft.setTextColor(GREY, BLACK);
    tft.setTextSize(1); //Smallest font size available
    tft.setCursor(0, 0); //x,y position to print text, at the bottom
    tft.print("DELTA VALUES-");
    tft.print(Time.format(Time.now(), "%l:%M:%S %p"));

    tft.copy_buffer(); //Nothing actually gets drawn on screen until we run this code, until now we were just filling display buffer
  attachInterrupt(D5, btnpush, FALLING);

  }
}


void windHandler(const char *event, const char *data)
{
      detachInterrupt(D5);

if (strstr(event, "Speed"))
 {
   strcpy (wind1, data);
   fwind = strtof (wind1, NULL);
   windsum += fwind;
   windcount ++;

 }
 else if(strstr(event, "Gust"))
 {
   strcpy (wind2, data);
 }

  attachInterrupt(D5, btnpush, FALLING);
}
*/
