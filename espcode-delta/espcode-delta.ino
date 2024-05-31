
//현재 이 프로그램에서 사용한 핀
//GPIO: 25,26,27,14
//UART : RX : 16, TX : 17
//Vin,GND  STM에 연결해야 제대로 작동함-전압이슈인듯

//official header
#include <WiFi.h>

#include <Ticker.h>
#include <WebServer.h>
#include <string.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <WiFiServer.h>
#include "ESPAsyncWebServer.h"

//custom header
#include "javascript.h"
#include "index.h"  

//OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Define
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64


//values
const char* ssid = "orugu";
const char* password = "bgct47264";
char* RX_Value = "";
char* received_value;
char* temp_value;
int count;
char test_value;
int updatecount=0;

//multiThreading
TaskHandle_t subtask;
TaskHandle_t main_task;

//Webserver
int page_num = 1;
AsyncWebServer server(80);
AsyncEventSource events("/events");
 
//OLED
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void InitWebServer()
{
  server.addHandler(&events);
  server.on("/",onRootRequest);
  
  server.on("/XFRONT",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(48);  //0
  request->send_P(200, "text/html", index_html, processor);
  Serial.println("XFront");
  });

  server.on("/XBACK",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(49);  //1
  request->send_P(200, "text/html", index_html, processor);
  Serial.println("XBack");
  });

  server.on("/YFRONT",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(50);  //2
  request->send_P(200, "text/html", index_html, processor);
  Serial.println("YFront");
  });

  server.on("/YBACK",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(51);  //3
  request->send_P(200, "text/html", index_html, processor);
  Serial.println("YBack");
  });

  server.on("/BALLUP",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(52);  //4
  request->send_P(200, "text/html", index_html, processor);
  Serial.println("BallUp");
  });

  server.on("/BALLDOWN",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(53);  //5
  request->send_P(200, "text/html", index_html, processor);
  Serial.println("BallDown");
  });

  server.begin();
}


//페이지 요청이 들어 오면 처리 하는 함수
String processor(const String& var){
  delay(100);
  return var;
}



void onRootRequest(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_html, processor);
}

static const unsigned long EVENT_INTERVAL_MS = 30;
void ProcessUpdate()
{
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + EVENT_INTERVAL_MS)
  {
      //이벤트 발송
    events.send(String(received_value).c_str(),"update", millis());
    prev_ms = millis();
    Serial.println("ProcessUpdated");
  }
}


//multitasking
void Subtask(void* pvParameter) {
  while(1)
  {
    Serial.println(test_value);
    test_value = Serial.read();
    delay(100);
  }

}
void Main_task(void* pvParameter)
{
  delay(10);
  while(1)
  {
    

    ProcessUpdate();
    if(updatecount%2==0)
    {
      //display.setCursor(40,10);
    }
    delay(10);
  }
  delay(50);
}

void setup() {
 // //display.clear//display();
  Serial.begin(115200);
  Serial.println("ESP32-TeraTerm Project");
  delay(100);
  Serial.println("-engine changed version-");
  delay(100);
  
//OLED
 // if(!//display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
  //p  Serial.println(F("SSD1306 allocation failed"));
  //  for(;;);
  //}

  //display.clearDisplay();
  //display.setTextSize(1);
  //display.setTextColor(WHITE);
  //display.setCursor(0,0);
  //display.println("ESP32-DEVKIT V1");
  //display.println("SYSTEM BOOTING");
  //display.display();
  delay(100);

  

  xTaskCreatePinnedToCore(Subtask, "subtask", 5000, NULL, 1, &subtask, 1);
  xTaskCreatePinnedToCore(Main_task, "main_task", 10000, NULL, 0, &main_task, 0);
  delay(100);
    
  //display.println(".....success");
  //display.display();

  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);
  Serial.println("STM32 Connection system booting");
  Serial2.begin(230400, SERIAL_7E1, 16, 17);
  Serial.println("completed");
  delay(100);

  // We start by connecting to a WiFi network

  Serial.print("WiFi Service booting");
  Serial.println("[network info]");
  Serial.print("ssid: ");
  Serial.println(ssid);
  delay(100);
  //display.print("ssid: ");
  //display.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //display.println("Boot Complete");
  delay(200);
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
 // display.clearDisplay();
 // display.setCursor(0,0);
  //display.println("ESP32-DEVKIT V1");
  //display.print("IP:");
  //display.println(WiFi.localIP());
  Serial.println("Web Server Initializing");
  InitWebServer();
  Serial.println("completed");

  // //display static text
  
  //display.//display();
  delay(100);
}


void loop() {
  delay(100);
 
}
