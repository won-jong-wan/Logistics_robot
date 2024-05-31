#include <WiFi.h>
#include <rrd.h>
#include <Ticker.h>
#include <WebServer.h>
#include <string.h>
#include <SPIFFS.h>
#include <FS.h>
#include <stdio.h>
#include <string.h>
#include <ArduinoJson.h>
#include <WiFiServer.h>
//JS 작성시 팁!
//항상 모든 줄 양 옆에 큰따옴표 표시!
//맨 끝에는 항상 ;!
int JS_NUM = 2;
//JS

String javascript_part_1 = 
              "function onloadFunc(){"
              "const mode = document.getElementById(\"mode\");"
              "mode.addEventListener(\"click\",function(){"
              "mode.innerText=\"";

String javascript_part_2=              
              "\";"
              "mode.onclick=function(){}});}"
              "window.onload=onloadFunc;";
/*
  "<form onSubmit=\"event.preventDefault(); testingresult(this);\">"
  "</form>"
*/
//result get
