#include <Arduino.h>
#line 1 "/home/cmach_socket/Arduino/TFT/TFT.ino"
#include "h.h"
#include "consts.h"
#include "xfont.h"
#include <TFT_eSPI.h>
#include <esp_pm.h>
#include "driver/temp_sensor.h"
#include<time.h>
#include <WiFiUdp.h>
#include<HTTPClient.h>
#include <NTPClient.h>
#include <WiFi.h>
#include<ArduinoJson.h>
#include<Preferences.h>
#include <Adafruit_BME680.h>
#include <SD.h>
#include <FS.h>
#include <JPEGDecoder.h>
Adafruit_BME680 bme; // I2C
struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};
TFT_eSPI tft = TFT_eSPI();
SPIClass sdSPI;
char pt1[20],pt2[20],pt3[20];
RTC_DATA_ATTR unsigned long change=0,changeTime=0;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int FLAG=0,lat=0,wific=0,turnoff=0,backlight=10;
File txtfile;
XFont *_xFont;
bool F=0,iswifi=0,nextpage=0;
float it=0,tp=0,pre=0,hu=0,gas=0,hi=0;
int tmpw=0,tmpw2=0,t5k=0,chosewin4=0,enterwin4=0,imagecnt=1,choseset=0,enterbook=0,bookchose=0,filecnt=0;
bool wincht,pagecht;
Button button2 = {0, 0, false};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp1.aliyun.com",60*60*8, 30*60*1000);
struct tm* tmp;
time_t tmp_time;
Preferences prefs;
char ptw[200]="";
String wre="",hitoreceive="",files[20];
#line 44 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void setup();
#line 111 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void loop();
#line 222 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void isr();
#line 229 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void winchange();
#line 235 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void pagechage();
#line 243 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void tempininit();
#line 251 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void sdinit();
#line 261 "/home/cmach_socket/Arduino/TFT/TFT.ino"
bool timeinit();
#line 270 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void bmeinit();
#line 281 "/home/cmach_socket/Arduino/TFT/TFT.ino"
int wificonnect();
#line 305 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void wifidis();
#line 314 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void gethito();
#line 325 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void setweather();
#line 363 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void get_time();
#line 370 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void get_tp();
#line 380 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void get_files();
#line 392 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void window1();
#line 404 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void window2();
#line 421 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void window3();
#line 428 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void window4();
#line 453 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void win4win1();
#line 458 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void win4win2();
#line 478 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void win4win3();
#line 483 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void win4win4();
#line 504 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void disply();
#line 542 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void showImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data);
#line 593 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void drawSdJpeg(const char *filename, int xpos, int ypos);
#line 627 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void jpegRender(int xpos, int ypos);
#line 44 "/home/cmach_socket/Arduino/TFT/TFT.ino"
void setup() {
  Wire.begin(16,17);
  Serial.begin(115200);
  esp_pm_config_esp32s3_t pmConfig;
  pmConfig.max_freq_mhz = 80;  // 最大工作频率（MHz）
  pmConfig.min_freq_mhz = 20;  // 最小工作频率（MHz）
  pmConfig.light_sleep_enable = true;  // 启用轻度睡眠模式
  esp_pm_configure(&pmConfig);
  tmp=new(tm);
  pinMode(button2.PIN, INPUT_PULLUP);
  attachInterrupt(button2.PIN, isr, FALLING);
  touchAttachInterrupt(T14, winchange, THRESHOLD);
  touchAttachInterrupt(T6, pagechage, THRESHOLD);
  pinMode(TFT_BL,OUTPUT);
  digitalWrite(TFT_BL,HIGH);
  sdinit();
  _xFont = new XFont();
  tft.init();
  ++bootCount;
  tft.fillScreen(TFT_BLACK);
  //tft.setRotation(1);
  tft.setCursor(0, 0, 2);
  prefs.begin("mynamespace");
  
  if(bootCount==1){
    tft.println(t5k);
    change=millis();
    while(1){
      if (pagecht) {
        t5k++;
        t5k%=5;
        tft.println(t5k);
        change=millis();
        pagecht = false;
      }
      if (wincht) {
        wific=t5k;
        if(t5k==4){
          timeval tmpt=(timeval){prefs.getUInt("time"),0};
          settimeofday(&tmpt,NULL);
          break;
        }
        if(wificonnect()){
          timeinit();
          setweather();
          wifidis();
          wincht=false;
          break;
        }
        else change=millis();
        wincht=false;
      }
    }
  }
  change=millis();
  tempininit();
  bmeinit();
  get_time();
  get_tp();
  touchSleepWakeUpEnable(T13,THRESHOLD);
  wre=prefs.getString("wre");
  disply();
  Serial.println(change);
  prefs.putUInt("time",time(0));
  Serial.println(wre);

}
void loop() {
  // put your main code here, to run repeatedly:
  
  if (millis() - changeTime > 10000) {
    changeTime = millis();
    get_time();
    if(tmp->tm_min!=lat){
      lat=tmp->tm_min;
      get_tp();
      disply();
    }

  }
  if (millis()-change > (turnoff*60+30)*1000 and !enterbook) {
    Serial.println("Going to sleep now");
    prefs.putUInt("time",time(0));
    //tft.fillScreen(TFT_BLACK);
    analogWrite(TFT_BL,0);
    showImage(0, 0, 135, 240, gp2);
    analogWrite(TFT_BL,0);
  
    wifidis();
    esp_deep_sleep_start();
  }
  if (wincht) {
    if(enterwin4){
      if(!chosewin4){
        imagecnt++;
      }
      else if(chosewin4==1){
        if(!enterbook){
          txtfile=SD.open("/books/"+files[bookchose]);
          enterbook=1;
        }
        nextpage=1;
      }
      else if(chosewin4==2){
        gethito();
      }
      else if(chosewin4==3){
        if(choseset==0){
          backlight=(backlight+1)%11;
        }
        else if(choseset==1){
          turnoff=(turnoff+1)%5;
        }
      }
    }
    else {  
      FLAG=(FLAG+1)%4;
    }
    Serial.printf("Button 2 has been pressed %u times\n", t5k);
    t5k++;
    disply();
    change=millis();
    wincht = false;
  }
  if (pagecht) {
    if(enterwin4){
      if(!chosewin4){
        imagecnt--;
      }
      else if(chosewin4==1){
        if(!enterbook)bookchose=(bookchose+1)%filecnt;
      }
      else if(chosewin4==3){
        choseset=(choseset+1)%2;
      }
    }
    else{
      if(!FLAG){
        wific=(wific+1)%4;
      }
      else if(FLAG==1){
        get_tp();
        get_time();
      }
      else if(FLAG==2){
        setweather();
      }
      else if(FLAG==3){
        chosewin4=(chosewin4+1)%4;
        
      }
    }
    disply();
    change=millis();
    pagecht=false;
  }
  if (button2.pressed) {
    if(!FLAG){
      if(!iswifi){
        wificonnect();
      }
      else{
        wifidis();
      }
      disply();
    }
    else if(FLAG==3){
      enterwin4=enterwin4^1;
      enterbook=filecnt=0;
      disply();
    }
    change=millis();
    button2.pressed=0;
  }
}
//main


void ARDUINO_ISR_ATTR isr() {
  if(millis()-change<300){
    return ;
  }
  button2.numberKeyPresses += 1;
  button2.pressed = true;
}
void winchange() {
  if(millis()-change<300){
    return ;
  }
  wincht = true;
}
void pagechage(){
  if(millis()-change<300){
     return ;
  }
  pagecht = true;
}
//buttons

void tempininit(){
   temp_sensor_config_t temp_sensor = {
    .dac_offset = TSENS_DAC_L2,
    .clk_div = 6,
  };
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
}
void sdinit(){
  pinMode(SD_CS,OUTPUT);
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  sdSPI.setClockDivider(SPI_CLOCK_DIV2);
  if (!SD.begin(SD_CS, sdSPI))
  {
    Serial.println("Failed to mount the memory card Procedure");
    return;
  }
}
bool timeinit(){
  timeClient.begin();
  while(timeClient.getEpochTime()<60*60*80){
    timeClient.update();
  }
  timeval tmpt=(timeval){timeClient.getEpochTime(),0};
  settimeofday(&tmpt,NULL);
  return 0;
}
void bmeinit(){
  bme.begin();
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}
//init


int wificonnect(){
  tft.println("start connect");
  Serial.println(wific);
  if(wific==1){
    WiFi.begin(id1,psw1);
  }
  else if(wific==2){
    WiFi.begin(id2,psw2);
  }
  else if(wific==3){
    WiFi.begin(id3,psw3);
  }
  else{
    return 0;
  }
  while(WiFi.status()!=WL_CONNECTED){      //未连接上
    delay(500);
    Serial.println("正在连接...");
  }
  Serial.println("连接成功！");        //连接上
  //digitalWrite(13,HIGH);
  iswifi=1;
  return 1;
}
void wifidis(){
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  //digitalWrite(13,LOW);
  iswifi=0;
}
//wifi


void gethito(){
  if(!iswifi){
    hitoreceive="NO WIFI CONNECT";
    return ;
  }
  HTTPClient http;
  http.begin(urlhito);
  int httpcode= http.GET();
  hitoreceive=http.getString();
  http.end();
}
void setweather(){
  if(!iswifi){
    return ;
  }
  HTTPClient http;
  http.begin(urlweahter);
  int httpcode= http.GET();
  String response=http.getString();
  http.end();
  DynamicJsonDocument doc(4096);
  deserializeJson(doc,response);
  tmpw=doc["result"]["realtime"]["wid"].as<int>();
  get_time();
  wre=pt2;
  wre+=" ";
  wre+=pt3;
  wre+="\n";
  wre+=wht[tmpw]+"\n";
  wre+="T:"+doc["result"]["realtime"]["temperature"].as<String>()+"*C ";
  wre+="H:"+doc["result"]["realtime"]["humidity"].as<String>()+"% ";
  wre+="A:"+doc["result"]["realtime"]["aqi"].as<String>()+"\n";
  for(int i=0;i<5;i++){
    wre+=doc["result"]["future"][i]["date"].as<String>()+"\n";
    wre+="T:"+doc["result"]["future"][i]["temperature"].as<String>();
    wre.setCharAt(wre.length()-3,' ');
    wre.setCharAt(wre.length()-2,'*');
    wre.setCharAt(wre.length()-1,'C');
    wre+="\n";
    tmpw=doc["result"]["future"][i]["wid"]["day"].as<int>();
    wre+=wht[tmpw];
    tmpw2=doc["result"]["future"][i]["wid"]["night"].as<int>();
    if(tmpw2!=tmpw){
      wre+=" to "+wht[tmpw2];
    }
    wre+="\n";
  }
  prefs.putString("wre",wre);
}
inline void get_time() {
  getLocalTime(tmp);
  strftime(pt1, 20, "%Y", tmp);
  strftime(pt2,20,"%m-%d %a",tmp);
  strftime(pt3, 20, "%H:%M", tmp);
  Serial.printf("%s %s %s\n",pt1,pt2,pt3);
}
inline void get_tp(){
  temp_sensor_read_celsius(&it);
  Serial.println(it);
  bme.performReading();
  tp=bme.temperature;
  pre=bme.pressure/100.0;
  hu=bme.humidity;
  gas=bme.gas_resistance/1000.0;
  hi=bme.readAltitude(SEALEVELPRESSURE_HPA);
}
inline void get_files(){
  File root = SD.open("/books");
  File file = root.openNextFile();
  while(file){
    files[filecnt]=file.name();
    file = root.openNextFile();
    filecnt++;
  }
}
//gets


void window1(){
  showImage(0, 0, 135, 240, gp2);
  tft.setCursor(10, 209, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(CC_DATUM);
  tft.print(wific);
  tft.print(" ");
  tft.println(iswifi);
  tft.setCursor(4, 220, 2);
  tft.println(pt3);
}
void window2(){
  showImage(0, 0, 135, 240, gp3);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(30, 0, 2);
  tft.setTextSize(2);
  tft.println(pt1);
  tft.setCursor(10,tft.getCursorY(),1);
  tft.println(pt2);

  tft.setCursor(3, 100, 6);
  tft.setTextSize(1);
  tft.println(pt3);

  tft.setCursor(3,190,2);
  tft.setTextSize(1);
  tft.printf("iT:%d*C sT:%.2f*C W:%d%% H:%dm G:%dKOhms P:%dhPa",int(it),tp,int(hu),int(hi),int(gas),int(pre));
}
void window3(){
  showImage(0, 0, 135, 240, gp3);
  tft.setCursor(0, 0, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.print(wre);
}
void window4(){
  showImage(0, 0, 135, 240, gp3);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(10,10,2);
  tft.setTextSize(1);
  tft.println("watchPic");

  tft.setCursor(10,30,2);
  tft.setTextSize(1);
  tft.println("watchEbook");

  tft.setCursor(10,50,2);
  tft.setTextSize(1);
  tft.println("hitokoto");

  tft.setCursor(10,70,2);
  tft.setTextSize(1);
  tft.println("settings");
  
  tft.setCursor(2,chosewin4*20+10,2);
  tft.print("*");
  //_xFont->DrawChinese(10, 10, "业精于勤荒于嬉戏，行成于思毁于随。业精于勤荒于嬉戏，行成于思毁于随。业精于勤荒于嬉戏，行成于思毁于随。业精于勤荒于嬉戏，行成于思毁于随。", 0);
  //tft.print(_xFont->isInit);
  
}
void win4win1(){
  char FileName[15];
  sprintf(FileName,"/pic/%d.jpg",imagecnt);
  drawSdJpeg(FileName, 0, 0);     // This draws a jpeg pulled off the SD Card
}
void win4win2(){
  tft.fillScreen(0xf7bb);
  tft.setTextColor(TFT_BLACK);
  if(!enterbook){
    if(!filecnt)get_files();
    for(int i=0;i<filecnt;i++){
      _xFont->DrawStr2(10,i*20+10,files[i],TFT_BLACK);
    }
    tft.setCursor(2,bookchose*20+10,2);
    tft.print("*");
    return;
  }
  if(!nextpage){
    return;
  }
  _xFont->DrawStrSelf(txtfile,TFT_BLACK);
  tft.setCursor(70,220,2);
  tft.print(txtfile.position()/(float)txtfile.size()*100);
  tft.print("%");
}
void win4win3(){
   showImage(0, 0, 135, 240, gp3);
  _xFont->DrawStr2(2, 10, hitoreceive, 0);
  
}
void win4win4(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  
  tft.setCursor(10,10,2);
  tft.setTextSize(1);
  tft.println("backlight");
  tft.setCursor(90,10,2);
  tft.println(25*backlight);

  tft.setCursor(10,30,2);
  tft.setTextSize(1);
  tft.println("turnoff");
  tft.setCursor(90,30,2);
  tft.println(turnoff*60+30);

  tft.setCursor(2,choseset*20+10,2);
  tft.print("*");
  tft.setCursor(4, 209, 2);
  tft.print("made by\ncmach_socket");
}
void disply(){
  //tft.fillScreen(TFT_BLACK);
  analogWrite(TFT_BL,backlight*25);
  tft.setSwapBytes(true);
  if(enterwin4){
    if(!chosewin4){
      win4win1();
    }
    else if(chosewin4==1){
      win4win2();
    }
    else if(chosewin4==2){
      win4win3();
    }
    else if(chosewin4==3){
      win4win4();
    }
    return ;
  }
  if(!FLAG){
    window1();
  }
  else if(FLAG==1){
    window2();
  }
  else if(FLAG==2){
    window3();
  }
  else if(FLAG==3){
    window4();
  }
  analogWrite(TFT_BL,backlight*25);
}
//dispaly



#define PI_BUF_SIZE 128
void showImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data){
  int32_t dx = 0;
  int32_t dy = 0;
  int32_t dw = w;
  int32_t dh = h*2;
 
  if (x < 0) { dw += x; dx = -x; x = 0; }
  if (y < 0) { dh += y; dy = -y; y = 0; }
 
  if (dw < 1 || dh < 1) return;
 
  CS_L;
 
  data += dx + dy * w;
 
  uint16_t  buffer[PI_BUF_SIZE];
  uint16_t* pix_buffer = buffer;
  uint16_t  high,low;
 
  tft.setWindow(x, y, x + dw - 1, y + dh - 1);
 
  // Work out the number whole buffers to send
  uint16_t nb = (dw * dh) / (2 * PI_BUF_SIZE);
 
  // Fill and send "nb" buffers to TFT
  for (int32_t i = 0; i < nb; i++) {
    for (int32_t j = 0; j < PI_BUF_SIZE; j++) {
      high = pgm_read_word(&data[(i * 2 * PI_BUF_SIZE) + 2 * j + 1]);
      low = pgm_read_word(&data[(i * 2 * PI_BUF_SIZE) + 2 * j ]);
      pix_buffer[j] = (high<<8)+low;
    }
    tft.pushPixels(pix_buffer, PI_BUF_SIZE);
  }
 
  // Work out number of pixels not yet sent
  uint16_t np = (dw * dh) % (2 * PI_BUF_SIZE);
 
  // Send any partial buffer left over
  if (np) {
    for (int32_t i = 0; i < np; i++)
    {
      high = pgm_read_word(&data[(nb * 2 * PI_BUF_SIZE) + 2 * i + 1]);
      low = pgm_read_word(&data[(nb * 2 * PI_BUF_SIZE) + 2 * i ]);
      pix_buffer[i] = (high<<8)+low;
    }
    tft.pushPixels(pix_buffer, np);
  }
 
  CS_H;
}
 
void drawSdJpeg(const char *filename, int xpos, int ypos) {
  uint32_t readTime = millis();
  // Open the named file (the Jpeg decoder library will close it)
  File jpegFile = SD.open( filename, FILE_READ);  // or, file handle reference for SD library
 
  if ( !jpegFile ) {
    Serial.print("ERROR: File \"");
    Serial.print(filename);
    Serial.println ("\" not found!");
    return;
  }
 
  Serial.println("===========================");
  Serial.print("Drawing file: "); Serial.println(filename);
  Serial.println("===========================");
 
  // Use one of the following methods to initialise the decoder:
  boolean decoded = JpegDec.decodeSdFile(jpegFile);  // Pass the SD file handle to the decoder,
  //boolean decoded = JpegDec.decodeSdFile(filename);  // or pass the filename (String or character array)
  if (decoded) {
    // print information about the image to the serial port
    // render the image onto the screen at given coordinates
    jpegRender(xpos, ypos);
  }
  else {
    Serial.println("Jpeg file format not supported!");
  }
}
 
//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void jpegRender(int xpos, int ypos) {
  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();
 
  //jpegInfo(); // Print information from the JPEG file (could comment this line out)
 
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;
 
  bool swapBytes = tft.getSwapBytes();
  tft.setSwapBytes(true);
  
  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = (mcu_w<(max_x % mcu_w)?mcu_w:(max_x % mcu_w));
  uint32_t min_h = (mcu_h<(max_y % mcu_h)?mcu_h:(max_y % mcu_h));
 
  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;
 
  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;
 
  // Fetch data from the file, decode and display
  while (JpegDec.read()) {    // While there is more data in the file
    pImg = JpegDec.pImage ;   // Decode a MCU (Minimum Coding Unit, typically a 8x8 or 16x16 pixel block)
 
    // Calculate coordinates of top left corner of current MCU
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;
 
    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
 
    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;
 
    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }
 
    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;
 
    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
      tft.pushImage(mcu_x, mcu_y, win_w, win_h, pImg);
    else if ( (mcu_y + win_h) >= tft.height())
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
  }
 
  tft.setSwapBytes(swapBytes);
 
  
}

