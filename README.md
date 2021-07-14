# Environmental_Rain_Gauge
BME280 outputs with Heatindex, Dewpoint; plus a rain gauge 

Features of "Environmental_Rain_Gauge.ino". 

1. NTP Time Server is used for 15 minute time interval, date-time stamping; Data log file gets created daily. 
File name is in the format "LOGxxyyzzzz" xx being the DATE and yy being the MONTH and zzzz being the YEAR.

2. Both (ESP32 Based and Domain, hosted) have dynamic web pages of current observations show Last update time and 
date, humidity, temperature, barometric pressure, rainfall by five minutes, hour, and day.

3. Data log files on server, are listed as web links and are displayed.

4. SERVER.TXT log each Server Start with date, time stamp.

5. WIFI.TXT logs WiFi Connects and WiFi Disconects with date, time stamp.   If WiFi disconnects; WiFi will 
reconnect after logging WiFi events. 

6. LOGXXYYZZZZ file is appended every 15 minutes with the latest update; sending data to Dynamic web pages; this 
file is renamed at the end of each day.

7. Sketch features FTP file transfer easing requirement for server maintenance.   Server can store a month of 
data log files.

8. Optional; two-line LCD Display of Barometric Pressure, in both inches of Mercury and millibars.

9. Temperature, Humidity, Barometric Pressure, and Dew Point have four embedded "ThinkSpeak.com" graphs on one 
web page. Graphs are created from Iframes provided by "ThingSpeak.com"

10. HTTPClient library is used to POST data to the Domain, hosted website. Function "webInterface" function sends data.

11. Free, "000webhost powered by HOSTINGER" may be used for Domain, hosted website.  Every 15 minutes; a new 
"Observations-II" web page is created.

12.  OTA updates are a feature of the Sketch. 

13. Two web sites, one sketch: "Environmental_Rain_Gauge.ino"

14.  Must maintain data log files by removing old log files, keep latest, four log files; easily accomplished using FTP.  
Filezillia has been used successfully.

ES32; Server http://weather-3.ddns.net/Weather  Dynamic, web page, file browser (selected LOG file can be viewed), and graphs can 
be displayed. 

ES32; Domain Server https://observations-weather.000webhostapp.com/observations-II.html Dynamic, web page.

Note this project is in development; maybe offline or log files may be affected. Server is online 24/7; except during 
periods of testing.

Server is a "HiLetGo,” ESP32 Development Board, a GY-BME280 breakout board, purchased from "Ebay.com," a NEO m8n GPS 
Module and a "tipping bucket" rain gauge are required for project.

Development board is "Arduino" friendly; can be programed using the "Arduino IDE."





