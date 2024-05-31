const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html> 
  <head>
    <meta charset="UTF-8">
    <title>Robot Console-supported by esp32</title>

    <script>
      if(!!window.EventSource){
        var source = new EventSource('/events');
        source.addEventListener('open',function(e){
          console.log("Events Connected");
        },false);
        source.addEventListener('update',function(e){
          var ad_value = e.data;
          document.getElementById("received_value").innerHTML=ad_value;
        },false);
      }
    </script>
  </head>
<body>
    <style>
    body{
    background-image:url('https://i.ibb.co/wSQ80sH/background-image.png');
    background-repeat:no-repeat;
    background-attachment: fixed;
    background-size :cover;
    }
    </style>
  <center>
  <h1>ESP32 Simple Web Server - engine restructured</h1> 
  </center>
  <p style= "font-size:5px;"><br>&nbsp;</p>
  <p style= "font-size:20px;">
    <!--주석입니다. 1st-->
    &nbsp;&nbsp;&nbsp;&nbsp;
    <a href="/START"><img src=https://i.ibb.co/xDzrjgG/power-on.png width ="100px" height ="100px"></a>&nbsp;&nbsp;&nbsp;
    <a href="/UP"><img src =https://i.ibb.co/C69m6M9/CW.png width ="100px" height ="100px"></a>&nbsp;&nbsp;&nbsp;
    <a href="/mode1"><img src=https://i.ibb.co/3TZF6Zw/mode-1.png alt=mode-1 border=0></a>&nbsp;
    <a href="/mode2"><img src=https://i.ibb.co/thMR83q/mode-2.png alt=mode-2 border=0></a>&nbsp;&nbsp;&nbsp;
    <a href="/TESTINGPAGE"><img src=https://i.ibb.co/cC9cVPb/dev-mode.png></a><br>

    <!--주석입니다. 2nd-->
    &nbsp;&nbsp;&nbsp;&nbsp;
    <a href="/STOP"><img src=https://i.ibb.co/fMNVbbm/power-off.png width = "100px" height = "100px"></a>&nbsp;&nbsp;&nbsp;
    <a href="/DOWN"><img src=https://i.ibb.co/68VRYNG/CCW.png width = "100px" height = "100px" CCW></a>&nbsp;&nbsp;&nbsp;
    <a href="/mode3"><img src=https://i.ibb.co/0rrKN8r/mode-3.png alt=mode-3 border=0></a>&nbsp;
    <a href="/mode4"><img src=https://i.ibb.co/PtWshWb/mode-4.png alt=mode-4 border=0></a>&nbsp;&nbsp;<br>
    
    <!--주석입니다. 3rd-->
    &nbsp;&nbsp;&nbsp;&nbsp;
    <a href="/STEPON"><img src=https://i.ibb.co/g6hZXRv/STEP-ON.png ></a>&nbsp;&nbsp;&nbsp;
    <a href="/STEPOFF"><img src=https://i.ibb.co/GQpcbqq/STEP-OFF.png></a>&nbsp;&nbsp;&nbsp;
    <a href="/PWM0"><img src=https://i.ibb.co/zN5xzWW/PWM0.png></a>&nbsp;
    <a href="/PWM5"><img src=https://i.ibb.co/PtXL4LV/PWM5.png></a>&nbsp;&nbsp;<br>

    <!--주석입니다. 4th-->
    &nbsp;&nbsp;&nbsp;&nbsp;
    <a href="/STEPCW"><img src =https://i.ibb.co/V2VHqDh/STEP-CW.png></a>&nbsp;&nbsp;&nbsp;
    <a href="/STEPCCW"><img src =https://i.ibb.co/4gkwrMS/STEP-CCW.png></a>&nbsp;&nbsp;&nbsp;
    <a href="/PWM20"><img src=https://i.ibb.co/x3vWfwn/PWM20.png></a>&nbsp;
    <a href="/PWM100"><img src=https://i.ibb.co/JtbmQ6d/PWM100.png alt=PWM100 border=0></a><br>
    &nbsp;
    status:
    <br>
    <button onClick="window.location.reload()">갱신</button>
    received value: <p id = "received_value">            </p>
    <br>
    battery value:
    <br>
    speed value:
    <br>
    position value:
    <br>

<h4>LED - Status <span id="outputState"><span></h4><label class="switch"><input type="checkbox" onchange="toggleCheckbox(this)" id="output" ><span class="slider"></span></label>
<script>
function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked)
  { 
      xhr.open("GET", "/update?output="+element.id+"&state=1", true);
  }
  else
  { 
      xhr.open("GET", "/update?output="+element.id+"&state=0", true);
  }
  xhr.send();
}
</script>


</body>
</html>
)rawliteral";
