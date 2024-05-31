//현재 이 프로그램에서 사용한 핀
//GPIO: 25,26,27,14
//UART : RX : 16, TX : 17
//Vin,GND  STM에 연결해야 제대로 작동함-전압이슈인듯

//official header


//custom header
#include "javascript.h"


//values
const char* ssid = "orugu";
const char* password = "bgct47264";
char* RX_Value = "";
WiFiServer server(80);
char received_value[9];
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


AsyncEventSource events
void createAndWriteFiles() {
  writeFile("/file1.txt", "File 1 in the house");
  writeFile("/file2.txt", "File 2 holds cabbages?!");
  writeFile("/file3.txt", "Content for file 3 is the biggest of them all!");
}

void writeFile(const char* path, const char* content) {
  Serial.print("Creating file: ");
  Serial.println(String(path));

  File file = SPIFFS.open(path, "w");
  if (file) {
    file.print(content);
    file.close();
    Serial.println("File created and written successfully");
  } else {
    Serial.println("Failed to create file");
  }
}

void listFiles(const char* dir) {
  Serial.print("Listing files in directory: ");
  Serial.println(String(dir));

  File root = SPIFFS.open(dir);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    Serial.println("File: " + String(file.name()) + ", Size: " + file.size());
    file = root.openNextFile();
  }
}

void Subtask(void* pvParameter) {
  while (1) {
    if (Serial.read() == 'A') {

      Serial.println("test completed");
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
    JS_NUM++;
    Serial.println(JS_NUM);
    delay(1000);
    if(JS_NUM>=1000)
    {
      JS_NUM=0;
      Serial.println("Counts resetted!");
      delay(1000);
    }
    server.listenOnLocalhost();
    if(server.hasClient()==true)
    {
    Serial.println("server handle keeped!");
    }
    else
    {
    Serial.println("server handle stopped!");
    }
    
  }
}
void Main_task(void* pvParameter)
{
  while(1)
  {
 WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                     // if you get a client,
    Serial.println("New Client.");  // print a message out the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's bytes to read from the client,
        char c = client.read();     // read a byte, then
        Serial.write(c);            // print it out the serial monitor
        if (c == '\n') {            // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

            if (page_num == 1) {
              //HTTP 헤더 관련
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              //폰트 크기 및 타이틀
              client.println("<font size=5>");
              client.print("<br>&nbsp;");
              client.println("<font size=20>");
              client.println("<title> TESTING</title>");
              // HTTP 내용관련
              // the content of the HTTP response follows the header:
              
              // 메타 설정
              client.print("<meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\" />");
              //client.print("<meta http-equiv=\"Refresh\" content=\"1\">");    //갱신이 필요할 경우 추가
              client.print("<style>\n body\{background-image\:url(\'https://i.ibb.co/wSQ80sH/background-image.png'); \n background-repeat\:no-repeat; \n background-attachment: fixed; \n background-size :100% 100%;</style>");
              
              //본 내용 띄우기
              


              //HTML 본 내용
              //hyperlink 파트
              //1st 
              client.print("<a href=\"/START\"> <img src=https://i.ibb.co/xDzrjgG/power-on.png width =\"100px\" height =\"100px\"></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/UP\"><img src =https://i.ibb.co/C69m6M9/CW.png width =\"100px\" height =\"100px\"></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/mode1\"><img src=https://i.ibb.co/3TZF6Zw/mode-1.png alt=mode-1 border=0></a>&nbsp;");
              client.print("<a href=\"/mode2\"><img src=https://i.ibb.co/thMR83q/mode-2.png alt=mode-2 border=0></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href =\"/TESTINGPAGE\"><img src=https://i.ibb.co/cC9cVPb/dev-mode.png></a><br>");
              
              //2nd
              client.print("&nbsp;");
              client.print("<a href=\"/STOP\"><img src=https://i.ibb.co/fMNVbbm/power-off.png width = \"100px\" height = \"100px\" motor stop></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/DOWN\"><img src=https://i.ibb.co/68VRYNG/CCW.png width = \"100px\" height = \"100px\" CCW></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/mode3\"><img src=https://i.ibb.co/0rrKN8r/mode-3.png alt=mode-3 border=0></a>&nbsp;");
              client.print("<a href=\"/mode4\"><img src=https://i.ibb.co/PtWshWb/mode-4.png alt=mode-4 border=0></a>&nbsp;&nbsp;<br>");
              
              //3rd
              client.print("&nbsp;");
              client.print("<a href=\"/STEPON\"><img src=https://i.ibb.co/g6hZXRv/STEP-ON.png ></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/STEPOFF\"><img src=https://i.ibb.co/GQpcbqq/STEP-OFF.png></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/PWM0\"><img src=https://i.ibb.co/zN5xzWW/PWM0.png></a>&nbsp;");
              client.print("<a href=\"/PWM5\"><img src=https://i.ibb.co/PtXL4LV/PWM5.png></a>&nbsp;&nbsp;<br>");
              
              //4th
              client.print("&nbsp;");
              client.print("<a href=\"/STEPCW\"><img src =https://i.ibb.co/V2VHqDh/STEP-CW.png></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/STEPCCW\"><img src =https://i.ibb.co/4gkwrMS/STEP-CCW.png></a>&nbsp;&nbsp;&nbsp;");
              client.print("<a href=\"/PWM20\"><img src=https://i.ibb.co/x3vWfwn/PWM20.png></a>&nbsp;");
              client.print("<a href=\"/PWM100\"><img src=https://i.ibb.co/JtbmQ6d/PWM100.png alt=PWM100 border=0></a><br>");
              
              //data received
              client.print("&nbsp;");
              client.print("status : ");
              client.print(status_now);
              client.print("<br>");

              //refresh button
              client.print("<button onClick=\"window.location.reload()\">갱신</button>");
              client.print("<br>");

              //HTTP input 내용 갱신
              //received value list
              client.print("&nbsp;");
              client.print("received value:");
              client.write(received_value[0]);
              client.write(received_value[1]);
              client.write(received_value[2]);
              client.write(received_value[3]);
              client.write(received_value[4]);
              client.write(received_value[5]);
              client.write(received_value[6]);
              client.write(received_value[7]);
              client.write(received_value[8]);
              client.write(received_value[9]);
              client.write(received_value[10]);
              client.write(received_value[11]);


              //1. 배터리
              Serial.write(received_value[0]);
              Serial.write(received_value[1]);
              value_1 = received_value[0];
              value_2 = received_value[1];
              client.print("<br>battery value: ");
              client.write(value_1);
              client.write(value_2);
              Serial.write(received_value[2]);
              Serial.write(received_value[3]);
              value_3 = received_value[2];
              value_4 = received_value[3];
              client.print(value_3);
              client.print(value_4);
              client.print("<br>");

              //2. 속도값
              client.print("speed value: ");
              Serial.write(received_value[4]);
              Serial.write(received_value[5]);
              value_5 = received_value[4];
              value_6 = received_value[5];
              client.print(value_5);
              client.print(value_6);
              Serial.write(received_value[6]);
              Serial.write(received_value[7]);
              value_7 = received_value[6];
              value_8 = received_value[7];
              client.print(value_7);
              client.print(value_8);
              client.print("<br>");
              //3. 위치값
              client.print("position value: ");
              Serial.println("");
              value_9 = received_value[8];
              client.print(value_9);
              value_10 = received_value[9];
              client.print(value_10);
              value_11 = received_value[10];
              client.print(value_11);
              value_12 = received_value[11];
              client.print(value_12);
              client.print("<br>");
              client.print("<div>");
              client.print("<button id= \"mode\">test</button>\n");
              client.print("</div>");
              // client.print(received_value);
              //뭔지 모름
              //
              //javascript test
              client.println("<script>");
              //javascript start
              client.print(javascript_part_1);
              client.print(JS_NUM);
              client.print(javascript_part_2);
              client.println("</script>");
              //jquery
              client.print("<script type=\'text/javascript\'>");
              client.println("function displaySensorValue(value){");
              client.println("var sensorDiv = document.getElementById(\"sensorValue\");");
              client.println("sensorDiv.innerHTML=\"센서 값: \"+ value;}\n");
              client.println("function updateDiv(){");
              client.println("async function fetchDataFromServer(){");
              client.println("try{const response = await fetch(\'/path/to/server\');");
              client.println("if(!response.ok){throw new Error(\'서버 응답 실패\');}");
              client.println("const data = await response.json();");
              client.println("displaySensorValue(data);}catch(error){console.error(\'오류 발생:\', error);}}");
              //페이지 로드 후 서버에서 데이터를 받아옴
              client.println("window.onload=function(){fetchDataFromServer();}};");
              
              client.println("</script>");
              

              client.println("</font>");
              client.println("</body>");
              client.println("</html>");
              //javascript ends
              // The HTTP response ends with another blank line:
              client.println();
              // break out of the while loop:
              break;
            }

            if (page_num == 2) {
              client.println("<font size=25>");
              client.println("<title> TESTING\/Testingpage</title>");
              // HTTP 내용관련
              // the content of the HTTP response follows the header:
              client.print("<meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\" />");
              client.print("<meta http-equiv=\"Refresh\" content=\"1\">");
              client.print("<body style = \"background-image\:url(\'https://images.unsplash.com/photo-1533035353720-f1c6a75cd8ab?q=80&w=987&auto=format&fit=crop&ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D\')\" /n background-repeat\:no-repeat>");
              client.print("Testboard for main page&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
              client.print("<a href =\"/return\">Main Page </a>");
              client.print("디버깅용 수신값: ");
              client.write(received_value[0]);
              client.write(received_value[1]);
              client.write(received_value[2]);
              client.write(received_value[3]);
              client.write(received_value[4]);
              client.write(received_value[5]);
              client.write(received_value[6]);
              client.write(received_value[7]);
              client.write(received_value[8]);
              client.write(received_value[9]);
              client.write(received_value[10]);
              client.write(received_value[11]);

              Serial.write(received_value[0]);
              Serial.write(received_value[1]);
              value_1 = received_value[0];
              value_2 = received_value[1];
              client.print("<br>battery value: ");
              client.write(value_1);
              client.write(value_2);
              Serial.write(received_value[2]);
              Serial.write(received_value[3]);
              value_3 = received_value[2];
              value_4 = received_value[3];
              client.print(value_3);
              client.print(value_4);
              client.print("<br>");

              //2. 속도값
              client.print("speed value: ");
              Serial.write(received_value[4]);
              Serial.write(received_value[5]);
              value_5 = received_value[4];
              value_6 = received_value[5];
              client.print(value_5);
              client.print(value_6);
              Serial.write(received_value[6]);
              Serial.write(received_value[7]);
              value_7 = received_value[6];
              value_8 = received_value[7];
              client.print(value_7);
              client.print(value_8);
              client.print("<br>");
              //3. 위치값
              client.print("position value: ");

              Serial.println("");
              value_9 = received_value[8];
              client.print(value_9);
              value_10 = received_value[9];
              client.print(value_10);
              value_11 = received_value[10];
              client.print(value_11);
              value_12 = received_value[11];
              client.print(value_12);

              client.print("<br>");
              break;
            }

          }

          else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        //동작용 논리-HTTP 내용관련에서 뒷부분 내용 따서 함
        // Check to see if the client request was "GET /H" or "GET /L":

        if (currentLine.endsWith("GET /UP")) {
          Serial2.write(50);  //2
          Serial2.write(50);
        }

        if (currentLine.endsWith("GET /DOWN")) {
          Serial2.write(51);  // 3
          Serial2.write(51);  //3
        }

        if (currentLine.endsWith("GET /START")) {
          Serial2.write(48);  //0
          Serial2.write(48);
        }

        if (currentLine.endsWith("GET /STOP")) {
          Serial2.write(49);  //1
          Serial2.write(49);
        }

        //PWM controller
        if (currentLine.endsWith("GET /PWM0")) {
          Serial2.write(52);
          Serial2.write(52);  //4
        }
        if (currentLine.endsWith("GET /PWM5")) {

          Serial2.write(53);
          Serial2.write(53);  //5
        }
        if (currentLine.endsWith("GET /PWM20")) {

          Serial2.write(54);
          Serial2.write(54);  //6
        }
        if (currentLine.endsWith("GET /PWM100")) {

          Serial2.write(55);
          Serial2.write(55);  //7
        }
        if (currentLine.endsWith("GET /mode1")) {

          Serial2.write(33);
          Serial2.write(33);  //!
        }

        if (currentLine.endsWith("GET /mode2")) {
          Serial2.write(34);
          Serial2.write(34);  //"
        }

        if (currentLine.endsWith("GET /mode3")) {
          Serial2.write(35);
          Serial2.write(35);  //#
        }
        if (currentLine.endsWith("GET /mode4")) {

          Serial2.write(36);
          Serial2.write(36);  //$
        }
        if (currentLine.endsWith("GET /STEPON")) {
          Serial2.write(65);
          Serial2.write(65);  //A
        }
        if (currentLine.endsWith("GET /STEPOFF")) {
          Serial2.write(66);
          Serial2.write(66);  //B
        }
        if (currentLine.endsWith("GET /STEPCW")) {
          Serial2.write(67);
          Serial2.write(67);  //C
        }
        if (currentLine.endsWith("GET /STEPCCW")) {
          Serial2.write(68);
          Serial2.write(68);  //D
        }
        if (currentLine.endsWith("GET /TESTINGPAGE")) {
          page_num = 2;
        }
        if (currentLine.endsWith("GET /return")) {
          page_num = 1;
        }
      }
    }
    server.listenOnLocalhost();
    //close the connection:
    //client.stop();
    //Serial.println("Client Disconnected.");
  }
  }
}

void setup() {
  Serial.begin(115200);


  if (SPIFFS.begin()) {
    Serial.println("SPIFFS ON");
    createAndWriteFiles();
    listFiles("/");
  } else {
    Serial.println("SPIFFS failed");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);

  Serial2.begin(230400, SERIAL_7E1, 16, 17);

  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  xTaskCreatePinnedToCore(Subtask, "subtask", 10000, NULL, 1, &subtask, 1);
  xTaskCreatePinnedToCore(Main_task, "main_task", 10000, NULL, 0, &main_task, 0);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.setNoDelay(true);
  server.begin();
  
}


void loop() {
 
}
