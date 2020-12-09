#ifndef webPage_h
#define webPage_h
#include "Arduino.h"

extern String Make_page();
const String WebPage2 = R"=====(
<!DOCTYPE html>
<html>
   <style>
      body { background-color: #e5ffcc; font-family: Arial, Helvetica, Sans-Serif; Color: #0080ff; }\
    </style>
   <style type="text/css">
textarea
{
  width: 300px;
  height: 40px;
  background-color: white;
  font-size: 1em;
  font-weight: bold;
  font-family: Verdana, Arial, Helvetica, sans-serif;
  border: 1px solid black;
}
  </style>
  <center>
<div id="demo">

<h1>ESP SENSOR STATION</h1>

</div>
 
<div>
Measurements:<br>
 <span id="Data">0
</span><br/>
</div>)=====";
const String WebPage2part2  = R"=====(
<script>
setInterval(function()
{
  // Call a function repetatively with 1 Second interval
  getData1();  
}, 1000); // 1000 mSeconds update rate

function getData1()
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() 
  {
    if (this.readyState == 4 && this.status == 200) 
    {
      document.getElementById("Data").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "takeData", true);
  xhttp.send();
}
</script>
</center>
</body>
</html>)=====";
#endif
