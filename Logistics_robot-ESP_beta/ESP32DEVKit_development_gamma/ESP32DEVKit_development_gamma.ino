//현재 이 프로그램에서 사용한 핀
//GPIO: 25,26,27,14
//UART : RX : 16, TX : 17
//Vin,GND  STM에 연결해야 제대로 작동함-전압이슈인듯

//official header
#include <WiFi.h>
#include <rrd.h>
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

//values
const char* ssid = "orugu";
const char* password = "bgct47264";
char* RX_Value = "";

char received_value[12];
char buffer[20];  //통신을 할때 buffer배열에 전송받은 데이터 입력
char bufferIndex = 0;
char value_1;
char value_2;
char value_3;
char value_4;
char value_5;
char value_6;
char value_7;
char value_8;
char value_9;
char value_10;
char value_11;
char value_12;
char* status_now = "off";
TaskHandle_t subtask;
TaskHandle_t main_task;
int page_num = 1;
AsyncWebServer server(80);
AsyncEventSource events("/events");
//WebServer 



void InitWebServer()
{
  server.addHandler(&events);
  server.on("/",onRootRequest);
  
  server.on("/UP",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(50);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/DOWN",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(51);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

    server.on("/START",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(48);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

    server.on("/STOP",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(49);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/PWM0",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(52);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

    server.on("/PWM5",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(53);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

    server.on("/PWM20",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(54);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

    server.on("/PWM100",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(55);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

    server.on("/mode1",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(33);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/mode2",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(34);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/mode3",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(35);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/mode4",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(36);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/STEPON",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(65);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/STEPOFF",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(66);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/STEPCW",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(67);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/STEPCCW",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  Serial2.write(68);  //2
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/TESTINGPAGE",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  
  request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/return",HTTP_GET, [] (AsyncWebServerRequest *request)
  {
  
  request->send_P(200, "text/html", index_html, processor);
  });
  server.begin();
}


//페이지 요청이 들어 오면 처리 하는 함수
String processor(const String& var){
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
  }
}


//multitasking
void Subtask(void* pvParameter) {
  while (1) {
    if (Serial.read() == 'A') {
      received_value[0]='1';
      received_value[1]='2';
      received_value[2]='3';
      received_value[3]='4';
      received_value[4]='5';
      received_value[5]='6';
      received_value[6]='7';
      received_value[7]='8';
      
      Serial.println("test completed");
    }
    if(Serial.read()== 'B')
    {
      received_value[0]='8';
      received_value[1]='7';
      received_value[2]='6';
      received_value[3]='5';
      received_value[4]='4';
      received_value[5]='3';
      received_value[6]='2';
      received_value[7]='1';
    }
    if ((Serial.read() == 63)) {
      received_value[0] = Serial.read();
      received_value[1] = Serial.read();
      received_value[2] = Serial.read();
      received_value[3] = Serial.read();
      received_value[4] = Serial.read();
      received_value[5] = Serial.read();
      received_value[6] = Serial.read();
      received_value[7] = Serial.read();
      received_value[8] = Serial.read();
      received_value[9] = Serial.read();
      received_value[10] = Serial.read();
      received_value[11] = Serial.read();
      
    }
    if (Serial2.available() && (Serial2.read() == 63)) {
      received_value[0] = Serial2.read();
      received_value[1] = Serial2.read();
      received_value[2] = Serial2.read();
      received_value[3] = Serial2.read();
      received_value[4] = Serial2.read();
      received_value[5] = Serial2.read();
      received_value[6] = Serial2.read();
      received_value[7] = Serial2.read();
      received_value[8] = Serial2.read();
      received_value[9] = Serial2.read();
      received_value[10] = Serial2.read();
      received_value[11] = Serial2.read();
      
    }

  }
}
void Main_task(void* pvParameter)
{
  delay(10);
  while(1)
  {

    ProcessUpdate();
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-TeraTerm Project");
  delay(100);
  Serial.println("-engine changed version-");
  delay(100);



  Serial.println("Multi-Tasking service booting");
  xTaskCreatePinnedToCore(Subtask, "subtask", 10000, NULL, 1, &subtask, 1);
  xTaskCreatePinnedToCore(Main_task, "main_task", 10000, NULL, 0, &main_task, 0);
  Serial.println("Multi-Tasking service booted successfully");


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

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("Web Server Initializing");
  InitWebServer();
  Serial.println("completed");
  delay(100);
}


void loop() {
 
}
