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
          var batteryvalue = ad_value.substring(0,4);
          document.getElementById("batteryvalue").innerHTML=batteryvalue;

          var speedvalue = ad_value.substring(4,8);
          document.getElementById("speedvalue").innerHTML=speedvalue;
          var positionvalue = ad_value.substring(8,12);
          document.getElementById("positionvalue").innerHTML=positionvalue;
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
    <a href="/XFRONT"><img src="https://i.ibb.co/wQgph28/Xaxis-Front.jpg" alt="Xaxis-Front" border="0" width ="100px" height ="100px"></a>&nbsp;&nbsp;&nbsp;
    <a href="/XBACK"><img src ="https://i.ibb.co/smbQStR/Xaxis-Back.jpg" width ="100px" height ="100px"></a>&nbsp;&nbsp;&nbsp;
    <a href="/YFRONT"><img src="https://i.ibb.co/VMCFXnS/Yaxis-Front.jpg" alt=mode-1 border=0></a>&nbsp;
    <a href="/YBACK"><img src="https://i.ibb.co/yYryPMZ/Yaxis-Back.jpg" alt=mode-2 border=0></a>&nbsp;&nbsp;&nbsp;
    
    <!--주석입니다. 2nd-->
    &nbsp;&nbsp;&nbsp;&nbsp;
    <a href="/BALLUP"><img src="https://i.ibb.co/x20M5Pb/BallUp.jpg" width = "100px" height = "100px"></a>&nbsp;&nbsp;&nbsp;
    <a href="/BALLDOWN"><img src="https://i.ibb.co/rG8X0fC/BallDown.jpg" width = "100px" height = "100px" CCW></a>&nbsp;&nbsp;&nbsp;
    <br>
    &nbsp;
    <p id = "ad_value">        </p>
    status:
    <br>
    <button onClick="window.location.reload()">갱신</button>

    

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
