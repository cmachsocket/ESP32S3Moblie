#define THRESHOLD 40
#define SEALEVELPRESSURE_HPA (1013.25)
#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13
#define BUT1        4
#define BUT2        19
#define PIN_INPUT1 4
#define PIN_INPUT2 19

const char* id1="TP-LINK_CEC3";   //定义两个字符串指针常量
const char* psw1="xxx123456";
const char* id2="ZYSZ";   //定义两个字符串指针常量
const char* psw2="xpxq999888";
const char* id3="AWSL";   //定义两个字符串指针常量
const char* psw3="1145141919810";

String urlweahter="http://apis.juhe.cn/simpleWeather/query?key=7623ab94560b9f163f0b9133f0c4f084&city=遵义";
String urlhito="https://v1.hitokoto.cn/?encode=text";