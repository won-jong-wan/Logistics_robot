//현재 이 프로그램에서 사용한 핀
//GPIO: 25,26,27,14
//UART : RX : 16, TX : 17
#include <WiFi.h>
#include <rrd.h>
#include <Ticker.h>
#include <WebServer.h>
#include <string.h>


const char* ssid     = "orugu";
const char* password = "bgct47264";
char* RX_Value = "";
WiFiServer server(80);
char received_value[8];
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


void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN,OUTPUT);
    delay(10);

    Serial2.begin(115200,SERIAL_7E1,16,17);
    
    // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(ssid);

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

            //HTTP 헤더 관련
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:

            client.println("<font size=20>");

            client.println("<title> TESTING</title>");

            // HTTP 내용관련
            // the content of the HTTP response follows the header:
            client.print("<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />");
            client.print("<a href=\"/START\"> <img src=https://static-00.iconduck.com/assets.00/power-symbol-emoji-457x512-37ndyg2l.png></a>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
            client.print("");

            client.print("<a href=\"/UP\"> CW</a><br>");
            

            client.print("<a href=\"/STOP\"> motor stop</a>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
            client.print("<a href=\"/DOWN\"> CCW</a><br>");
            
            client.print("<a href=\"/PWM0\">PWM=0</a>&nbsp;&nbsp;      ");
            client.print("<a href=\"/PWM5\">PWM=5</a>&nbsp;&nbsp;     ");
            client.print("<a href=\"/PWM20\">PWM=20</a>&nbsp;&nbsp;   ");
            client.print("<a href=\"/PWM100\">PWM=100</a> <br>      ");
            client.print("<a href=\"/mode1\">mode1-33</a>&nbsp;&nbsp;      ");
            client.print("<a href=\"/mode2\">mode2-34</a>&nbsp;&nbsp;      ");
            client.print("<a href=\"/mode4\">mode4-36</a>&nbsp;&nbsp;<br>      ");
            client.print("<button onClick=\"window.location.reload()\">refresh</button>");
            //HTTP input 내용 갱신

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
              }
            
            
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
            //1. 전류값 1
            Serial.write(received_value[0]);
            Serial.write(received_value[1]);
            value_1 = received_value[0];
            value_2 = received_value[1];
            client.print("<br>Amp1 value: ");            
            client.write(value_1);
            client.write(value_2);
            client.print("<br>");

            //2. 전류값 2
            client.print("Amp2 value: ");
            Serial.write(received_value[2]);
            Serial.write(received_value[3]);
            value_3 = received_value[2];
            value_4 = received_value[3];
            client.print(value_3);
            client.print(value_4);  
            client.print("<br>");
            
            //3. 속도값 1
            client.print("speed value 1:");
            Serial.write(received_value[4]);
            Serial.write(received_value[5]);
            value_5 = received_value[4];
            value_6 = received_value[5];
            client.print(value_5);
            client.print(value_6);
            client.print("<br>");
            
            //4. 속도값 2
            client.print("speed value 2:");
            Serial.write(received_value[6]);
            Serial.write(received_value[7]);
            value_7 = received_value[6];
            value_8 = received_value[7];
            client.print(value_7);
            client.print(value_8);
            client.print("<br>");
           // client.print(received_value);
            //뭔지 모름 
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        //동작용 논리-HTTP 내용관련에서 뒷부분 내용 따서 함
        // Check to see if the client request was "GET /H" or "GET /L":

        if (currentLine.endsWith("GET /UP")) {
        Serial2.write(50);                 //q
        Serial2.write(50);
        }

        if (currentLine.endsWith("GET /DOWN")) {
        Serial2.write(51);                // 후진
        Serial2.write(51);                 //f
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
        Serial2.write(52);                //5
        }
        if (currentLine.endsWith("GET /PWM5")) {

        Serial2.write(53);
        Serial2.write(53);                //6
        }
        if (currentLine.endsWith("GET /PWM20")) {

        Serial2.write(54);
        Serial2.write(54);                //7
        }
        if (currentLine.endsWith("GET /PWM100")) {

        Serial2.write(55);
        Serial2.write(55);                //8
        }
        if (currentLine.endsWith("GET /mode1")) {

        Serial2.write(33);
        Serial2.write(33);                //2
        }

        if (currentLine.endsWith("GET /mode2")) {
        Serial2.write(34);
        Serial2.write(34);                //2
        }

        if (currentLine.endsWith("GET /mode3")) {
        Serial2.write(35);
        Serial2.write(35);                //2
        }
        if (currentLine.endsWith("GET /mode4")) {

        Serial2.write(36);
        Serial2.write(36);                //2
        }
    
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
