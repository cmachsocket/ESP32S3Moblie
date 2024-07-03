#include "h.h"
#include "consts.h"
#include "xfont.h"
#include <TFT_eSPI.h>
#include <esp_pm.h>
#include <OneButton.h>
//#include "driver/temp_sensor.h"
#include<time.h>
#include <WiFiUdp.h>
#include<HTTPClient.h>
#include <NTPClient.h>
#include <WiFi.h>
#include<ArduinoJson.h>
#include<Preferences.h>
//#include "bsec.h"
#include <SD.h>
#include <FS.h>
#include <JPEGDecoder.h>

//Bsec iaqSensor;
OneButton button1(PIN_INPUT1,true,true);
OneButton button2(PIN_INPUT2, true,true);
TFT_eSPI tft = TFT_eSPI();
SPIClass sdSPI;
char pt1[20],pt2[20],pt3[20];
RTC_DATA_ATTR unsigned long change=0,changeTime=0;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int FLAG=0,lat=0,wific=0,turnoff=1,backlight=10;
File txtfile,bookmark;
XFont *_xFont;
bool F=0,iswifi=0,nextpage=0,lastpage=0,thefirstenter=0;
float it=0,tp=0,pre=0,hu=0,gas=0,hi=0,co2=0,iaq=0;
int tmpw=0,tmpw2=0,t5k=0,chosewin4=0,enterwin4=0,imagecnt=1,choseset=0,enterbook=0,bookchose=0,filecnt=0,stop=0,isiaq=0,doupress=0,sek=1;
uint32_t lastpos=0,llpos=0;
bool wincht,pagecht;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp1.aliyun.com",60*60*8, 30*60*1000);
struct tm* tmp;
time_t tmp_time;
Preferences prefs;
char ptw[200]="";
String wre="",hitoreceive="",files[20],wrestr[20];
void setup() {
  //Wire.begin(16,17);
  
  Serial.begin(115200);
  tmp=new(tm);
  button1.attachClick(click1);
  button1.attachDoubleClick(doubleclick1);
  
  button2.attachClick(click2);
  pinMode(TFT_BL,OUTPUT);
  digitalWrite(TFT_BL,HIGH);
  sdinit();
  _xFont = new XFont();
  tft.init();
  ++bootCount;
  tft.fillScreen(TFT_BLACK);
  //tft.setRotation(1);
  prefs.begin("mynamespace");
  checkinit();
  change=millis();
  tempininit();
  //bmeinit();
  get_time();
  //get_tp();
  
  
  wre=prefs.getString("wre");
  for(int i=0;i<wre.length();i++){
    if(wre[i]=='\n'){
      sek++;
      continue;
    }
    wrestr[sek]+=wre[i];
  }
  gpio_hold_dis(GPIO_NUM_32);
  disply();
  analogWrite(TFT_BL,backlight*25);
  Serial.println(change);
  prefs.putUInt("time",time(0));
  Serial.println(wre);

}
void loop() {
  button1.tick();
  button2.tick();
  // put your main code here, to run repeatedly:
  //if(iaqSensor.run()){
    //get_tp();
  //}
  if (millis() - changeTime > 10000) {
    changeTime = millis();
    get_time();
    if(tmp->tm_min!=lat){
      lat=tmp->tm_min;
      disply();
    }

  }
  if (turnoff and !enterbook and millis()-change > (turnoff*60)*1000 ) {
    Serial.println("Going to sleep now");
    prefs.putUInt("time",time(0));
    //
    wifidis();
    analogWrite(TFT_BL,0);
    tft.fillScreen(TFT_BLACK);
    analogWrite(TFT_BL,0);
    gpio_hold_en(GPIO_NUM_32);
    touchSleepWakeUpEnable(T0,THRESHOLD);
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
          bookmark=SD.open("/bookmark/"+files[bookchose]);
          llpos = bookmark.read()<<24;
          llpos += (bookmark.read()<<16);
          llpos += (bookmark.read()<<8);
          llpos += bookmark.read();
          bookmark.close();
          enterbook=1;
          thefirstenter=1;
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
        else if(choseset==2){
          esp_restart();
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
        else lastpage=1;
      }
      else if(chosewin4==3){
        choseset=(choseset+1)%3;
      }
      
    }
    else{
      if(!FLAG){
        wific=(wific+1)%4;
      }
      else if(FLAG==1){
        //get_tp();
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
  if (doupress) {
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
      if(enterbook){
        bookmark = SD.open("/bookmark/"+files[bookchose],FILE_WRITE);
        bookmark.write((lastpos>>24)&0xFF);
        bookmark.write((lastpos>>16)&0xFF);
        bookmark.write((lastpos>>8)&0xFF);
        bookmark.write((lastpos)&0xFF);
        bookmark.close();
        enterbook=filecnt=0;
      }
      disply();
    }
    change=millis();
    doupress=0;
  }
}
//main


void doubleclick1() {
  if(millis()-change<300){
    return ;
  }
  doupress = true;
}
void click1() {
  Serial.println(">>");
  wincht = true;
}
void click2(){
  Serial.println("LL");
  pagecht = true;
}
//buttons
void checkinit(){
  if(bootCount==1){
    tft.setCursor(100, 20, 2);
    tft.println(t5k);
    change=millis();
    while(1){
      Serial.println(digitalRead(PIN_INPUT2));
      button1.tick();
      button2.tick();
      
      if (pagecht) {
        t5k++;
        t5k%=5;
        tft.setCursor(100,tft.getCursorY());
        tft.println(t5k);
        change=millis();
        pagecht = false;
      }
      if (wincht) {
        wific=t5k;
        tft.setCursor(100,tft.getCursorY());
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
      delay(10);
    }
  }
}

void tempininit(){
  //  temp_sensor_config_t temp_sensor = {
  //   .dac_offset = TSENS_DAC_L2,
  //   .clk_div = 6,
  // };
  // temp_sensor_set_config(temp_sensor);
  // temp_sensor_start();
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
  // iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
  // checkIaqSensorStatus();
  // bsec_virtual_sensor_t sensorList[13] = {
  //   BSEC_OUTPUT_IAQ,
  //   BSEC_OUTPUT_STATIC_IAQ,
  //   BSEC_OUTPUT_CO2_EQUIVALENT,
  //   BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
  //   BSEC_OUTPUT_RAW_TEMPERATURE,
  //   BSEC_OUTPUT_RAW_PRESSURE,
  //   BSEC_OUTPUT_RAW_HUMIDITY,
  //   BSEC_OUTPUT_RAW_GAS,
  //   BSEC_OUTPUT_STABILIZATION_STATUS,
  //   BSEC_OUTPUT_RUN_IN_STATUS,
  //   BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
  //   BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  //   BSEC_OUTPUT_GAS_PERCENTAGE
  // };
  // iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
}
//init


int wificonnect(){
  tft.println("start connect");
  Serial.println(wific);
  get_fre(80);
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
  get_fre(20);
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
  //temp_sensor_read_celsius(&it);
  Serial.println(it);
  // tp=iaqSensor.temperature;
  // pre=iaqSensor.pressure/100.0;
  // hu=iaqSensor.humidity;
  // gas=iaqSensor.gasPercentage;
  // co2=iaqSensor.co2Equivalent;
  // isiaq=iaqSensor.iaqAccuracy;
  // iaq=iaqSensor.iaq;
  //hi=calAltitude(pre,tp);
}
inline float calAltitude(float atmospheric , float temprature){
 return  (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903))*(temprature+273.15)/0.0065;
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
inline void get_fre(int mhz){
  esp_pm_config_esp32_t pmConfig;
  pmConfig.max_freq_mhz = mhz;  // 最大工作频率（MHz）
  pmConfig.min_freq_mhz = mhz;  // 最小工作频率（MHz）
  pmConfig.light_sleep_enable = true;  // 启用轻度睡眠模式
  esp_pm_configure(&pmConfig);
}
//gets


void window1(){
  tft.pushImage(0,0,240,240,gp2);
  tft.setCursor(34, 190, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(CC_DATUM);
  tft.print(wific);
  tft.print(" ");
  tft.println(iswifi);
  tft.setCursor(27, 180, 2);
  tft.println(pt3);
}
void window2(){
  tft.pushImage(0,0,240,240,gp3);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(85, 20, 2);
  tft.setTextSize(2);
  tft.println(pt1);
  tft.setCursor(60,tft.getCursorY(),1);
  tft.println(pt2);

  tft.setCursor(50, 130, 6);
  tft.setTextSize(1);
  tft.println(pt3);

  //tft.setCursor(3,170,2);
  //tft.setTextSize(1);
  //tft.printf("iT:%d*C sT:%.2f*C W:%d%% I:%d H:%dm G:%d%% P:%dhPa C:%dppm",int(it),tp,int(hu),int(isiaq ? iaq : 0 ),int(hi),int(gas),int(pre),int(co2));
}
void window3(){
  tft.pushImage(0,0,240,240,gp3);
  tft.setCursor(70, 20, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  for(int i=1;i<sek;i++){
    tft.setCursor(50,tft.getCursorY(), 2);
    tft.println(wrestr[i]);
  }
}
void window4(){
  tft.pushImage(0,0,240,240,gp3);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(70,10,2);
  tft.setTextSize(1);
  tft.println("watchPic");

  tft.setCursor(70,30,2);
  tft.setTextSize(1);
  tft.println("watchEbook");

  tft.setCursor(70,50,2);
  tft.setTextSize(1);
  tft.println("hitokoto");

  tft.setCursor(70,70,2);
  tft.setTextSize(1);
  tft.println("settings");
  
  tft.setCursor(62,chosewin4*20+10,2);
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
  if(!enterbook){
    tft.fillScreen(0xf7bb);
    tft.setTextColor(TFT_BLACK);
    if(!filecnt)get_files();
    for(int i=0;i<filecnt;i++){
      _xFont->DrawStr(10,i*20+10,files[i],TFT_BLACK);
    }
    tft.setCursor(2,bookchose*20+10,2);
    tft.print("*");
    return;
  }
  if(!nextpage and !lastpage){
    return;
  }
  tft.fillScreen(0xf7bb);
  tft.setTextColor(TFT_BLACK);
  if(thefirstenter){
    tft.setCursor(10,50,2);
    tft.print("wincht : from head \n pagecht : from mark");
    thefirstenter=0;
    return;
  }
  if(lastpage){
    txtfile.seek(max((unsigned)(0),llpos));
  }
  llpos=lastpos;
  lastpos=txtfile.position();
  _xFont->DrawStrSelf(txtfile,TFT_BLACK);
  tft.setCursor(10,220,2);
  tft.print(pt3);
  tft.setCursor(70,220,2);
  tft.print(txtfile.position()/(float)txtfile.size()*100);
  tft.print("%");
  lastpage=nextpage=0;
}
void win4win3(){
   tft.pushImage(0,0,240,240,gp3);
  _xFont->DrawStr(10, 100, hitoreceive, 0);
  
}
void win4win4(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  
  tft.setCursor(70,10,2);
  tft.setTextSize(1);
  tft.println("backlight");
  tft.setCursor(150,10,2);
  tft.println(25*backlight);

  tft.setCursor(70,30,2);
  tft.setTextSize(1);
  tft.println("turnoff");
  tft.setCursor(150,30,2);
  tft.println(turnoff*60);
  
  tft.setCursor(70,50,2);
  tft.setTextSize(1);
  tft.println("restart");

  tft.setCursor(62,choseset*20+10,2);
  tft.print("*");
  tft.setCursor(104, 160, 2);
  tft.println("made by");
  tft.setCursor(84, tft.getCursorY(), 2);
  tft.println("cmach_socket");
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
