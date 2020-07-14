#include <TrueRMS.h>
#include <UIPEthernet.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define LPERIOD 2000    // loop period time in us. In this case 2.0ms
#define RMS_WINDOW 80 
#define PORTUDP 6000
String DEVICE_ID = "105";

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
unsigned long nextLoop;
int cnt = 0;
OneWire oneWire(2); //Podłączenie czujnika temperatury do CYFROWEGO 2
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,16,2) for 16x2 LCD.
EthernetClient client;
EthernetUDP udp;

Power acL1;
Power acL2;
Power acL3;

bool tick=true;
int licz=0;


//FFT

#include "arduinoFFT.h"
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 200; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

//


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}


void(* resetFunc) (void) = 0;


void setup() {  // run once:
  Serial.begin(115200);
  
  Serial.println("");
  Serial.println("----------- ARDUINO - URUCHAMIANIE -----------");
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Arduino"); 
  lcd.setCursor(0, 1); lcd.print("Uruchamianie");
  delay(1500);
  lcd.setCursor(0, 0); lcd.print("LACZENIE Z      ");
  lcd.setCursor(0, 1); lcd.print("INTERNETEM      ");

  //Serial.println("Akcelerometr");


  sensors.begin();
  sensors.setResolution(10);

  if (!accel.begin())
  {
    Serial.println("Problem z wykryciem akcelerometru ADXL345! Sprawdz polaczenie!");
    delay(1500);
    lcd.setCursor(0, 0); lcd.print("BLAD            ");
    lcd.setCursor(0, 1); lcd.print("AKCELEROMETRU   ");
    delay(2000);
    lcd.setCursor(0, 0); lcd.print("RESTART ZA      ");
    lcd.setCursor(0, 1); lcd.print("5 SEKUND        ");
    delay(5000);
  }


  accel.setRange(ADXL345_RANGE_8_G);
  //displaySensorDetails();
  //displayDataRate();
  //displayRange();
  Serial.println("");


  byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

  if (Ethernet.begin(mac) == 0) {
    Serial.println("----------- PROBLEM Z KONFIGURACJĄ ETHERNETU -----------");                               //TRZEBA OBSŁUŻYĆ!!!

    lcd.setCursor(0, 0); lcd.print("Internet:       ");
    lcd.setCursor(0, 1); lcd.print("BLAD POLACZENIA!");
    delay(2000);
    lcd.setCursor(0, 0); lcd.print("RESTART ZA      ");
    lcd.setCursor(0, 1); lcd.print("5 SEKUND        ");
    delay(5000);
    resetFunc();
  } else {

    lcd.setCursor(0, 0); lcd.print("Internet: OK    ");
    lcd.setCursor(0, 1); lcd.print(Ethernet.localIP());
    delay(2000);
  }

  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  delay(1500);
  lcd.setCursor(0, 0); lcd.print("ROZPOCZYNAM      ");
  lcd.setCursor(0, 1); lcd.print("PRACE           ");

  
  acL2.begin(650, 50, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  acL2.start();

  acL1.begin(650, 50, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  acL1.start();

  acL3.begin(650, 50, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  acL3.start();
 
  nextLoop = micros() + LPERIOD;
}



void loop() {
  
  //brazowa (1) - prad a13 a14 a15 napiecie chyba a10 a11 a12
  //czarna (2) - A3 A2
  //szara (3) - A0 A1

 //Odczyt napięć:
    
  int  acVolt1 = analogRead(A10);
  if (acVolt1 <= 493 && acVolt1 >= 491) acVolt1 = 510;
    else acVolt1 = map(analogRead(A10), 220, 688, 0, 1023);

  int acVolt2 = analogRead(A11);
  if (acVolt2 <= 491 && acVolt2 >= 490) acVolt2 = 510;
    else acVolt2 = map(analogRead(A11), 219, 684, 0, 1023);

  int acVolt3 = analogRead(A12);
  if (acVolt3 <= 489 && acVolt3 >= 488) acVolt3 = 510;
    else acVolt3 = map(analogRead(A12), 219, 686, 0, 1023);


  //Odczyt prądów:


  int acCurr1 = analogRead(A13);
  if (acCurr1 <= 496 && acCurr1 >= 489) acCurr1 = 510;

  int acCurr2 = analogRead(A14);
  if (acCurr2 <= 494 && acCurr2 >= 487) acCurr2 = 510;

  int acCurr3 = analogRead(A15);
  if (acCurr3 <= 493 && acCurr3 >= 486) acCurr3 = 510;
  
  acL1.update(acVolt1, acCurr1);
  acL2.update(acVolt2, acCurr2);
  acL3.update(acVolt3, acCurr3);

  cnt++;
  if (cnt >= 150) { // co sekunde

    sensors_event_t event; 
    accel.getEvent(&event);

    sensors.requestTemperatures(); //Pobranie temperatury czujnika
    float temperatura = sensors.getTempCByIndex(0);

    String mystr;
    mystr = String(temperatura);

    mystr += " \337C         ";

    if(tick){
      tick=false;
      lcd.setCursor(0, 0); lcd.print("TEMPERATURA:    ");
    }else{
      lcd.setCursor(0, 0); lcd.print("TEMPERATURA:   *");
      tick=true;
    }
    lcd.setCursor(0, 1); lcd.print(mystr);

    acL1.publish();
    acL2.publish();
    acL3.publish();

    //Serial.println("-----------------------------------------------");


    /*
      Serial.print("V1: ");
      Serial.print(acL1.rmsVal1, 2);
      Serial.print(", I: ");
      Serial.print(acL1.rmsVal2, 2);
      Serial.print(", Moc pozorna: ");
      Serial.print(acL1.apparentPwr, 1);
      Serial.print(", Moc czynna: ");
      Serial.print(acL1.realPwr, 1);
      Serial.print(", wsp. mocy: ");
      Serial.print(acL1.pf, 2);
      Serial.print(", energia: ");
      Serial.println(acL1.energy / 3600, 2); // Wh

      Serial.print("V2: ");
      Serial.print(acL2.rmsVal1, 2);
      Serial.print(", I: ");
      Serial.print(acL2.rmsVal2, 2);
      Serial.print(", Moc pozorna: ");
      Serial.print(acL2.apparentPwr, 1);
      Serial.print(", Moc czynna: ");
      Serial.print(acL2.realPwr, 1);
      Serial.print(", wsp. mocy: ");
      Serial.print(acL2.pf, 2);
      Serial.print(", energia: ");
      Serial.println(acL2.energy / 3600, 2); // Wh

      Serial.print("V3: ");
      Serial.print(acL3.rmsVal1, 2);
      Serial.print(", I: ");
      Serial.print(acL3.rmsVal2, 2);
      Serial.print(", Moc pozorna: ");
      Serial.print(acL3.apparentPwr, 1);
      Serial.print(", Moc czynna: ");
      Serial.print(acL3.realPwr, 1);
      Serial.print(", wsp. mocy: ");
      Serial.print(acL3.pf, 2);
      Serial.print(", energia: ");
      Serial.println(acL3.energy / 3600, 2); // Wh
    */


    //--------------------------------------------------------
    // WYSYŁANIE 

    String temp = "#"+DEVICE_ID+"#";    //arduino id
    temp += acL1.rmsVal2;   //prąd fazy 1
    temp += "#";
    temp += acL2.rmsVal2;   //prąd fazy 2
    temp += "#";
    temp += acL3.rmsVal2;   //prąd fazy 3
    temp += "#";

    temp += temperatura;          //temperatura silnika
    temp += "#";
    temp += acL1.rmsVal1;   //napiecie fazy 1
    temp += "#";
    temp += acL2.rmsVal1;   //napiecie fazy 2
    temp += "#";
    temp += acL3.rmsVal1;   //napiecie fazy 3

    temp += "#";
    temp += acL1.realPwr;   //moc czynna fazy 1
    temp += "#";
    temp += acL2.realPwr;   //moc czynna fazy 2
    temp += "#";
    temp += acL3.realPwr;   //moc czynna fazy 3

    temp += "#";
    temp += acL1.apparentPwr - acL1.realPwr; //moc bierna fazy 1
    temp += "#";
    temp += acL2.apparentPwr - acL2.realPwr; //moc bierna fazy 2
    temp += "#";
    temp += acL3.apparentPwr - acL3.realPwr; //moc bierna fazy 3

    temp += "#";
    temp += acL1.apparentPwr;   //moc pozorna fazy 1
    temp += "#";
    temp += acL2.apparentPwr;   //moc pozorna fazy 2
    temp += "#";
    temp += acL3.apparentPwr;   //moc pozorna fazy 3


    if (isnan(acL1.pf)) acL1.pf = 0;
    if (isnan(acL2.pf)) acL2.pf = 0;
    if (isnan(acL3.pf)) acL3.pf = 0;

    temp += "#";
    temp += acL1.pf;   //cos fazy 1
    temp += "#";
    temp += acL2.pf;   //cos fazy 2
    temp += "#";
    temp += acL3.pf;   //cos fazy 3

    temp += "#";
    temp += acL1.energy/3600;   //energia fazy 1
    temp += "#";
    temp += acL2.energy/3600;   //energia fazy 2
    temp += "#";
    temp += acL3.energy/3600;   //energia fazy 3
    
    temp += "#";
    temp += event.acceleration.x;   //akcel x
    temp += "#";
    temp += event.acceleration.y;   //akcel y
    temp += "#";
    temp += event.acceleration.z;   //akcel z


   char c[temp.length()];
   temp.toCharArray(c, sizeof(c));

   udp.beginPacket(IPAddress(136,243,156,120),PORTUDP);
   udp.write(c);
   udp.endPacket();

   udp.beginPacket(IPAddress(212,91,26,112),6001);
   udp.write(c);
   udp.endPacket();
   
   //udp.flush();
   //udp.stop();





    licz++;
    if(licz==200)
      licz=125; 
    
    if(licz==35){
      
      lcd.setCursor(0, 0); lcd.print("WYKONUJE POMIAR ");
      lcd.setCursor(0, 1); lcd.print("WIBRACJI FFT    ");

      funcFFT("fftx");
      delay(1000);
      funcFFT("ffty");
      delay(1000);
      funcFFT("fftz");
      delay(1000);
    }
    
/*
    if (client.connect("izo.ct8.pl", 80)) { 136.243.156.120 192,168,0,101

      temp = "data=" + temp;

      client.println("POST /post.php HTTP/1.1");
      client.println("Host: izo.ct8.pl");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.println("Connection: close");
      client.println("User-Agent: Arduino/1.0");
      client.print("Content-Length: ");
      client.println(temp.length());
      client.println();
      client.print(temp);
      client.println();
      client.stop();

    }
    */

    //--------------------------------------------------------


  cnt = 0;

  }

  while(nextLoop > micros());  // wait until the end of the time interval
  nextLoop += LPERIOD;  // set next loop time to current time + LOOP_PERIOD
}


void funcFFT(String txt)
{
  /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {

    sensors_event_t event; 
    accel.getEvent(&event);


    if(txt=="fftx"){
      vReal[i] = event.acceleration.x;
    }
    if(txt=="ffty"){
      vReal[i] = event.acceleration.y;
    }
    if(txt=="fftz"){
      vReal[i] = event.acceleration.z;
    }

    /*
    if(txt=="i1"){
      vReal[i] = event.acceleration.z;
    }
    if(txt=="i2"){
      vReal[i] = event.acceleration.z;
    }
*/

    
      //vReal[i] = event.acceleration.x;
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }


  /* Print the results of the sampling according to time */
 // Serial.println("Data:");
 // PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
 // Serial.println("Weighed data:");
 // PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
 // Serial.println("Computed Real values:");
  //PrintVector(vReal, samples, SCL_INDEX);
 // Serial.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY, txt);
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //Serial.println(x, 6); //Print out what frequency is the most dominant.

}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType, String txt)
{

  String log="#"+DEVICE_ID+"-"+txt+"#";
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    abscissa = ((i * 1.0 * samplingFrequency) / samples);

    log+="#";
   // Serial.print(abscissa, 6);        
    log+=abscissa;
    if(scaleType==SCL_FREQUENCY)
    //  Serial.print("Hz");             
      log+="Hz ";
   // Serial.print(" ");
   // Serial.println(vData[i], 4);      
    log+=vData[i];
    
  }
  
  Serial.println(log);
  
   char c[log.length()];
   log.toCharArray(c, sizeof(c));

   udp.beginPacket(IPAddress(136,243,156,120),PORTUDP);
   udp.write(c);
   udp.endPacket();


   //udp.beginPacket(IPAddress(212,91,26,112),6001);//212.91.26.112
  // udp.write(c);
   //udp.endPacket();

   
   //udp.stop();
/*


    if (client.connect("panel-obslugi.pl", 80)) { // 192,168,0,101

      String temp = "data=" + log;

      client.println("POST /fft.php HTTP/1.1");
      client.println("Host: izo.ct8.pl");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.println("Connection: close");
      client.println("User-Agent: Arduino/1.0");
      client.print("Content-Length: ");
      client.println(temp.length());
      client.println();
      client.print(temp);
      client.println();
      client.stop();

    }*/

}
