//현재 이 프로그램에서 사용한 핀
//GPIO: 25,26,27,14
//UART : RX : 16, TX : 17
//Vin,GND  STM에 연결해야 제대로 작동함-전압이슈인듯

#include <WiFi.h>
#include <rrd.h>
#include <Ticker.h>
#include <WebServer.h>
#include <string.h>
#include <SPIFFS.h>
#include <FS.h>

const char* ssid     = "orugu";
const char* password = "bgct47264";
char* RX_Value = "";
WiFiServer server(80);
char received_value[9];
char buffer[20];               //통신을 할때 buffer배열에 전송받은 데이터 입력
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
int page_num = 1;

void createAndWriteFiles() {
  writeFile("/file1.txt", "File 1 in the house");
  writeFile("/file2.txt", "File 2 holds cabbages?!");
  writeFile("/file3.txt", "Content for file 3 is the biggest of them all!");
}

void writeFile(const char *path, const char *content) {
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

void listFiles(const char *dir) {
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

void Subtask(void *pvParameter)
{
  while(1)
  {
    if (Serial.read()=='A')
    {

      Serial.println("test completed");
    }
    if(Serial2.available()&&(Serial2.read()==63))
      {
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


void setup()
{
    Serial.begin(115200);


    if(SPIFFS.begin()){
      Serial.println("SPIFFS ON");
      createAndWriteFiles();
      listFiles("/");
    }
    else
    {
      Serial.println("SPIFFS failed");
    }
    
    pinMode(LED_BUILTIN,OUTPUT);
    delay(10);

    Serial2.begin(230400,SERIAL_7E1,16,17);
    
    // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(ssid);
    xTaskCreatePinnedToCore(Subtask,"subtask",10000,NULL,0,&subtask,1);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    server.begin();
}


void loop(){
 WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

            if(page_num ==1){
            //HTTP 헤더 관련
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("<font size=20>");
            client.println("<title> TESTING</title>");
            // HTTP 내용관련
            // the content of the HTTP response follows the header:
            client.print("<meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\" />");
            //client.print("<meta http-equiv=\"Refresh\" content=\"1\">");    //갱신이 필요할 경우 추가
            client.print("<body style = \"background-image\:url(\'https://images.unsplash.com/photo-1533035353720-f1c6a75cd8ab?q=80&w=987&auto=format&fit=crop&ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D\')\" /n background-repeat\:no-repeat>");
            client.print("<a href=\"/START\"> <img src=https://cdn.discordapp.com/attachments/1214089641125347379/1214092023758590052/power_on.png?ex=65f7da4f&is=65e5654f&hm=fc3261729481be99021ab19a9c68211dde73c677cce4f92e2bf81fc9b48f89f2& width =\"100px\" height =\"100px\"></a>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/UP\"><img src =https://cdn.discordapp.com/attachments/1214089641125347379/1216634375962361876/CW..png?ex=66011a0f&is=65eea50f&hm=63f33085cc57b6a1e4b01b541e296f24cc35aa3fae2bb47e9a5beb9209b92147& width =\"100px\" height =\"100px\"></a>&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/mode1\"><img src =https://cdn.discordapp.com/attachments/1214089641125347379/1214098069247893555/mode_1.png?ex=65f7dff0&is=65e56af0&hm=a65e4bbae9fe1ad21683193ad50370b8844b58e83214af7ef5ca34ca77a33d03&></a>&nbsp;");
            client.print("<a href=\"/mode2\"><img src = https://cdn.discordapp.com/attachments/1214089641125347379/1214098069537431552/mode_2.png?ex=65f7dff0&is=65e56af0&hm=8e3ba0172e97ead629cb9c68dddf97f26ab0bd33da6294fda3e94454191cb221&></a><br>");
            client.print("<a href=\"/STOP\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1214091898831110204/power_off.png?ex=65f7da31&is=65e56531&hm=f1f06ee5da21765b9dcebf10848040cd0076539772a59ed84027565e23dca2e7& width = \"100px\" height = \"100px\" motor stop></a>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/DOWN\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1216634374829637745/CCW.png?ex=66011a0e&is=65eea50e&hm=ad781e96cbf538fde99a37454d95f417c7f352a8a8b2c9488b21a21de38ceae6& width = \"100px\" height = \"100px\" CCW></a>&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/mode3\"><img src =https://cdn.discordapp.com/attachments/1214089641125347379/1214098069830766592/mode_3.png?ex=65f7dff0&is=65e56af0&hm=db8447af4956d60b1106809c2e95cc3da0dca338c1061b695b29351a5e9818d0&></a>&nbsp;");
            client.print("<a href=\"/mode4\"><img src =https://cdn.discordapp.com/attachments/1214089641125347379/1214098070115975188/mode_4.png?ex=65f7dff0&is=65e56af0&hm=580e7b36db05d0c07054dffff80377f8da9eb4dafd98c9563a1926ec93443f7a&></a>&nbsp;&nbsp;<br>      ");
            client.print("<a href=\"/STEPON\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1216630557161820201/STEP_ON.png?ex=66011680&is=65eea180&hm=260d2d63c81fd11f6fafdcf9b6ef06f2fa287d6801ce223c574168ce558d21d8& ></a>&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/STEPOFF\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1216630558113927178/STEP_OFF.png?ex=66011680&is=65eea180&hm=26a7b638b62d82330b6f6b7d89ab3ff1853609f80a5aab4bb1d8ed7177a48637&></a>&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/PWM0\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1217103129732976691/PWM0.png?ex=6602ce9e&is=65f0599e&hm=870d6cf862e9e5539e3d1647325b14338ac1441587dad645d5d1747092b84666&></a>&nbsp;&nbsp;");
            client.print("<a href=\"/PWM5\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1217103130714443957/PWM5.png?ex=6602ce9f&is=65f0599f&hm=53e6c82e707bd7e518cd3cd030c0d3e63859ba49e6869c0873e0a44485639440&></a>&nbsp;&nbsp;<br>");
            client.print("<a href=\"/STEPCW\"><img src =https://cdn.discordapp.com/attachments/1214089641125347379/1216630557862137876/STEP_CW.png?ex=66011680&is=65eea180&hm=9c15af0c1d26606aaeedaa7520736f103ea0958cc81267a75ac721317e3be4bf& ></a>&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/STEPCCW\"><img src =https://cdn.discordapp.com/attachments/1214089641125347379/1216630557568532480/STEP_CCW.png?ex=66011680&is=65eea180&hm=f7c8aacb0f2d0a327fbe3c659eb6c2ff7c970d7adff987c2265637ae62986bf7&  ></a>&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/PWM20\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1217103130270109786/PWM20.png?ex=6602ce9e&is=65f0599e&hm=13b5cea91a9d432c0784c301d1445890babf60a8d92312318c8029396cb0549d&></a>&nbsp;&nbsp;");
            client.print("<a href=\"/PWM100\"><img src=https://cdn.discordapp.com/attachments/1214089641125347379/1217103131184464104/PWM100.png?ex=6602ce9f&is=65f0599f&hm=708b2647903e42161dc51fb64671b25371fac20c7418bf7096194f608ae779dc&></a><br>");
            client.print("status : ");
            client.print(status_now);
            client.print("<br>"); 
            client.print("<button onClick=\"window.location.reload()\">갱신</button>");
            client.print("<a href =\"/TESTINGPAGE\">TESTINGPAGE</a><br>");
            //HTTP input 내용 갱신
            //received value list
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
           // client.print(received_value);
            //뭔지 모름 
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
            }

            if (page_num ==2)
            {
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

          else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        //동작용 논리-HTTP 내용관련에서 뒷부분 내용 따서 함
        // Check to see if the client request was "GET /H" or "GET /L":

        if (currentLine.endsWith("GET /UP")) {
        Serial2.write(50);                 //2
        Serial2.write(50);
        }

        if (currentLine.endsWith("GET /DOWN")) {
        Serial2.write(51);                // 3
        Serial2.write(51);                 //3
        }

        if (currentLine.endsWith("GET /START"))
        {
        Serial2.write(48);                  //0
        Serial2.write(48);
        
        }

        if (currentLine.endsWith("GET /STOP")) {
        Serial2.write(49);                 //1
        Serial2.write(49);
        }

        //PWM controller
        if (currentLine.endsWith("GET /PWM0")) {
        Serial2.write(52);
        Serial2.write(52);                //4
        }
        if (currentLine.endsWith("GET /PWM5")) {

        Serial2.write(53);
        Serial2.write(53);                //5
        }
        if (currentLine.endsWith("GET /PWM20")) {

        Serial2.write(54);
        Serial2.write(54);                //6
        }
        if (currentLine.endsWith("GET /PWM100")) {

        Serial2.write(55);
        Serial2.write(55);                //7
        }
        if (currentLine.endsWith("GET /mode1")) {

        Serial2.write(33);
        Serial2.write(33);                //!
        }

        if (currentLine.endsWith("GET /mode2")) {
        Serial2.write(34);
        Serial2.write(34);                //"
        }

        if (currentLine.endsWith("GET /mode3")) {
        Serial2.write(35);
        Serial2.write(35);                //#
        }
        if (currentLine.endsWith("GET /mode4")) {

        Serial2.write(36);
        Serial2.write(36);                //$
        }
        if (currentLine.endsWith("GET /STEPON")) {
        Serial2.write(65);
        Serial2.write(65);                //A
        }
        if (currentLine.endsWith("GET /STEPOFF")) {
        Serial2.write(66);
        Serial2.write(66);                //B
        }
        if (currentLine.endsWith("GET /STEPCW")) {
        Serial2.write(67);
        Serial2.write(67);                //C
        }
        if (currentLine.endsWith("GET /STEPCCW")) {
        Serial2.write(68);
        Serial2.write(68);                //D
        }
        if (currentLine.endsWith("GET /TESTINGPAGE"))
        {
          page_num =2;
        }
        if (currentLine.endsWith("GET /return"))
        {
          page_num = 1;
        }

      }
    }
    // close the connection:
    //client.stop();
    Serial.println("Client Disconnected.");
  }
}