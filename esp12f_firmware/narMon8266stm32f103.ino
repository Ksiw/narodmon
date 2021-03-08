#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
//-------------------------------------------------------------------------------------------------------
#include <NTPClient.h>
#include <time.h>
#include <WiFiUdp.h>
#define postingInterval 301000 //5мин 1 сек
#define debug true    // вывод отладочных сообщений
//-------------------------------------------------------------------------------------------------------
WiFiUDP ntpUDP;
const long utcOffsetInSeconds = 10800;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
static char temperatureCString[20];

static String bufTime;
static unsigned long lastConnectionTime = 0; // время последней передачи данных
static String Hostname;            //имя железки - выглядит как ESPAABBCCDDEEFF т.е. ESP+mac адрес.
static int RXindex = 0;

static unsigned long lastSendTime = 0; // время последней передачи данных
static unsigned long lastSendTimeInterval = 3600000;
static bool f_startRx = false;

//static int idxTemperat = 0;
bool f_temperat = false;
static float temperatur;

void readTemp();
void starting();
void Sendtime();
//-------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10);
  //WiFi.mode(WIFI_STA);
  pinMode(LED_BUILTIN, OUTPUT);
  Hostname = "ESP" + WiFi.macAddress();
  Hostname.replace(":", "");
  WiFi.hostname(Hostname);
  wifimanstart();
  //Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());
  Serial.print("Narodmon ID: ");
  Serial.println(Hostname);
  lastConnectionTime = millis() - postingInterval + 15000; //первая передача на народный мониторинг через 15 сек.
  starting();
  /*
    while (1)
    {
    display.clearDisplay();
    display.drawPixel(random(0, 128), random(0, 32), WHITE);
    display.display();
    delay(10);
    }*/
  timeClient.update();
  Sendtime();
  timeClient.setUpdateInterval(3600000);
}
//-------------------------------------------------------------------------------------------------------
void loop()
{
  if (millis() - lastConnectionTime > postingInterval)
  { // ждем 5 минут и отправляем
    if (WiFi.status() == WL_CONNECTED)
    {
      if (SendToNarodmon())
        lastConnectionTime = millis();
      else
        lastConnectionTime = millis() - postingInterval + 15000; //следующая попытка через 15 сек
    }
    else
    {
      lastConnectionTime = millis() - postingInterval + 15000; //следующая попытка через 15 сек
      Serial.println("Not connected to WiFi");
    }
  }

  if (millis() - lastSendTime > lastSendTimeInterval)
  {
    Sendtime();
    lastSendTime = millis();
  }

  readTemp();
  yield(); // системная работа wi-fi
  //delay(500);
}
//-------------------------------------------------------------------------------------------------------
void wifimanstart() // точку доступа ESP8266 и настроечную таблицу http://192.168.4.1
{
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(debug);
  wifiManager.setMinimumSignalQuality();
  if (!wifiManager.autoConnect("ESP8266"))
  {
    delay(3000);
    ESP.reset();
    delay(5000);
  }
}
//-------------------------------------------------------------------------------------------------------
bool SendToNarodmon()
{
  if (f_temperat)
  {
    // Собственно формирование пакета и отправка.
    WiFiClient client;
    String buf;
    buf = "#" + Hostname + "\r\n";
    buf = buf + "#TEMPC";
    memset(temperatureCString, 0, sizeof(char) * 6);
    dtostrf(temperatur, 2, 1, temperatureCString);
    buf = buf + "#" + String(temperatureCString) + "\r\n"; //и температура
    buf = buf + "##\r\n";

    if (!client.connect("narodmon.ru", 8283))
    {
      Serial.println("connection failed");
      return false;
    }
    else
    {
      client.print(buf); // и отправляем данные
      if (debug)
        Serial.print(buf);
      while (client.available())
      {
        String line = client.readStringUntil('\r'); // если что-то в ответ будет - все в Serial
      }
    }
    f_temperat = false;
    return true; //ушло
  }
}

//-------------------------------------------------------------------------------------------------------
void readTemp()
{
//  Serial.println(Serial.available());
  if (Serial.available())
  {
    while (Serial.available())
    {
      temperatureCString[RXindex] = Serial.read();
      if (temperatureCString[RXindex] == '$')
      {
        temperatur = Serial.parseFloat();          
        if (temperatur > -40 && temperatur < 70)
        {
          f_temperat = true;
          Serial.println(temperatur);
        }
        else
          f_temperat = false;
      }
    }
  }  
}
//------------------------------------------------------------------------------------------------------
void Sendtime()
{ //timeClient.update();
  if (WiFi.status() == WL_CONNECTED)
  {
    static unsigned long epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime ((time_t *)&epochTime);
    //$ деньмесяца, месяц, год, час, минута, секунда * crc  memset(bufTime, 0, bufTime.length());
    bufTime = "$";
    bufTime += timeClient.getDay();
    bufTime += ";";
    bufTime += ptm->tm_mday;
    bufTime += ";";
    bufTime += ptm->tm_mon + 1;;
    bufTime += ";";
    bufTime += ptm->tm_year + 1900 - 2000;
    bufTime += ";";
    bufTime += timeClient.getHours();
    bufTime += ";";
    bufTime += timeClient.getMinutes();
    bufTime += ";";
    bufTime += timeClient.getSeconds();
    bufTime +=  '\n';
    Serial.print(bufTime);
  }
}
//-------------------------------------------------------------------------------------------------------
void starting()
{
  timeClient.begin();
}
