#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_RESET 0
#define SSRX D5
#define SSTX D6
#define GPS_BAUD 9600
#define POS_LINE 0
#define SAT_LINE 12
#define TIME_LINE 24
#define FST_COL 0
#define SCD_COL 64

Adafruit_SSD1306 oled(OLED_RESET);

int sats = 0, dayMonthYear;
float latitude = 0, longitude = 0;
double nPITI_lat = -5.842586, nPITI_lon = -35.197695;

TinyGPSPlus gps;
//TinyGPSCustom satsInView(gps, "GPGSV", 3);
SoftwareSerial ss(SSRX, SSTX);

void feed();
void updateDisplay();
void printPosition();
void printFloats(float var);
void printSats();
void printHMS();
void printDate();
void printDistanceToNPITI();
static void smartDelay(unsigned long ms);

void setup() {

  ss.begin(GPS_BAUD);
  oled.begin();
  oled.clearDisplay();

}

void loop() {

  feed();
  updateDisplay();
  smartDelay(1000);
}

void feed()
{
  sats = gps.satellites.value();
  latitude = gps.location.lat();
  longitude = gps.location.lng();
  dayMonthYear = gps.date.value();
}

void updateDisplay()
{
  String displayOutput;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);

 printPosition();

printSats();

  printDate();

  printHMS();

  printDistanceToNPITI();

  oled.display();

}

void printPosition()
{
  if(!gps.location.isValid()){
    oled.setCursor(FST_COL, POS_LINE);
    oled.print("Aguardando conexao...");
    return;
  }
 oled.setCursor(FST_COL, POS_LINE);
  printFloats(latitude);

  oled.setCursor(SCD_COL, POS_LINE);
  printFloats(longitude);
}

void printFloats(float var)
{
  String Output;
  char str[32];
  sprintf(str, "%f", var);
  Output = str;
  oled.print(Output);
}

void printSats(){
  String Output = String(sats);
  Output += " sats";
  oled.setCursor(FST_COL, SAT_LINE);
  oled.print(Output);
}


void printHMS()
{
  String HMS;
  HMS = String(gps.time.hour());
  HMS += ":";
  HMS += String(gps.time.minute());
  HMS += ":";
  HMS += String(gps.time.second());
  oled.setCursor(FST_COL, TIME_LINE);
  oled.print(HMS);
}

void printDate()
{
  String DMY;
  DMY = String(gps.date.day());
  DMY += "/";
  DMY += String(gps.date.month());
  DMY += "/";
  DMY += String(gps.date.year() - 2000);
  oled.setCursor(SCD_COL, TIME_LINE);
  oled.print(DMY);
}

void printDistanceToNPITI()
{
  if (!gps.location.isValid()) return;
  unsigned long distanceToNPITI =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      nPITI_lat,
      nPITI_lon);
  oled.setCursor(SCD_COL, SAT_LINE);
  oled.print(distanceToNPITI);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
