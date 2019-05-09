
// utilizar para fila -> Se inscrever na propia key pra saber ser foi realmente enviado (Dica Ricardo)

#include <Arduino.h>
#include <TinyGPS++.h>                // Codificação da info do GPS
#include <SoftwareSerial.h>           // Leitura do módulo GPS pela serial
#include <Adafruit_SSD1306.h>         // Monitor OLED
#include <Adafruit_GFX.h>             // Monitor OLED
#include <SaIoTDeviceLib.h>           // Comunicação entre dispositivos e sistema SAIOT
#include <QueueList.h>                // Gerenciamento de filas FIFO
#include <Ticker.h>                   // Interrupções internas

#define OLED_RESET 0
#define SSRX D5
#define SSTX D6
#define GPS_BAUD 9600
#define POS_LINE 0
#define SAT_LINE 12
#define TIME_LINE 24
#define FST_COL 0
#define SCD_COL 64

class data{                       // Dados coletados a serem postos em fila
  public: 
  float value;                    // Valor
  unsigned long instant;          // Momento da coleta
};


// GLOBAIS
int sats = 0, dayMonthYear;
float latitude = 0, longitude = 0;
double nPITI_lat = -5.842586, nPITI_lon = -35.197695;   // Posição do lab p calc de dist
unsigned long distanceToNPITI;
String senha = "12345678910";

// OBJETOS
TinyGPSPlus gps;                            // GPS
SoftwareSerial ss(SSRX, SSTX);              // Serial de software
Adafruit_SSD1306 oled(OLED_RESET);          // Monitor OLED
WiFiClient espClient;                       // Cliente WiFi
SaIoTDeviceLib barril("Barril","060519LABB","ricardo@email.com");   // Dispositivo(nome, serial, email)
SaIoTSensor lat("lat","Latitude","Graus","number");                 // Sensor(key, tag, unidade, tipo)
SaIoTSensor lon("lon","Longitude","Graus","number"); 
SaIoTSensor distancia("dist","Distancia Npiti","m","number"); 
SaIoTSensor nSatelites("satelites","Satelites","qnt","number"); 
QueueList<data> ValueQ;                                             // Fila de dados

// DECLARAÇÃO DE FUNÇÕES
void feed();
void updateDisplay();
void printPosition();
void printFloats(float var);
void printSats();
void printHMS();
void printDate();
void printDistanceToNPITI();
static void smartDelay(unsigned long ms);
void callback(char* topic, byte* payload, unsigned int length);
void sendData2Saiot();

void setup() {

  barril.addSensor(lat);         // inicialização de sensores do dispositivo
  barril.addSensor(lon);
  barril.addSensor(distancia);
  barril.addSensor(nSatelites);

  Serial.begin(115200);
  Serial.println("INICIO");
  ss.begin(GPS_BAUD);

  barril.preSetCom(espClient, callback);    // Função padrão obrigatória (?)
  barril.start(senha);                      // Inicialização da com. entre dispositivo e sistema SAIOT

  
  oled.begin();
  oled.clearDisplay();

}

void loop() {

  smartDelay(1000);
  feed();
  updateDisplay();
  sendData2Saiot();
  barril.handleLoop();
}

void sendData2Saiot(){                        // Enviar dados
  String dateTime = SaIoTCom::getDateNow();
  lat.sendData((latitude),dateTime);
  lon.sendData((longitude),dateTime);
  nSatelites.sendData((sats),dateTime);
  distancia.sendData((distanceToNPITI),dateTime);
}

void feed()                                   // Extrair info do sensor GPS
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

void printPosition()                          //
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
  distanceToNPITI =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      nPITI_lat,
      nPITI_lon);
  oled.setCursor(SCD_COL, SAT_LINE);
  oled.print(distanceToNPITI);
}

static void smartDelay(unsigned long ms)      // Por 1 segundo, codifica mensagens do GPS
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void callback(char* topic, byte* payload, unsigned int length){    // Função padrão obrigatória (?)
  String payloadS;
  Serial.print("Topic: ");
  Serial.println(topic);
  for (unsigned int i=0;i<length;i++) {
    payloadS += (char)payload[i];
  }
  if(strcmp(topic,barril.getSerial().c_str()) == 0){
    Serial.println("SerialLog: " + payloadS);
  }
  // if(strcmp(topic,(barril.getSerial()+solenoide.getKey()).c_str()) == 0){
  //   Serial.println("SerialLog: " + payloadS);
  //   //
  // } 
}

void FuncPush()
{
  dados d;
  d.valor = cont;
  d.time = millis() / 1000;
  filaPulsos.push(d);
  cont = 0;
}