//Arduino PCB加热台3.1 for esp32
//添加ADC校准功能
//负熵生之光 2021 9 10

#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3C for 128x64, 0x3D for 128x32
#define SDA_PIN 18
#define SCL_PIN 19
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


const char *ServerName = "ESP32-PCB_HEAT";
const char *ssid = "ESP32_PCB_HEAT";
const char *password = "esp12345678";

byte dy = 26;                   //电压检测PIN
byte dl = 34;                   //电流检测PIN
byte pr = 25;                   //热电阻检测PIN
byte k1 = 22;                    //按键1检测PIN
byte k2 = 23;                    //按键2检测PIN
byte k3 = 16;                    //按键3检测PIN
byte k4 = 4;                    //按键4检测PIN

byte adc = 33;                  //2.5V基准电压检测PIN
int TL25 = 2500;                //TL431基准电压mv
int VCC = 3300;                 //MCU电源电压mv
int adc_OFF_set = 0;            //ADC偏差值mv

byte mos1 = 21;                  //mos控制PIN
byte BUZZER_PIN = 5;            //蜂鸣器
byte BUZZER_CHANNEL = 0;        //蜂鸣器

float V;                        //电压
float A;                        //电流
float W;                        //功率
float Z;                        //负载电阻	通过电流及电压计算所得
float R;                        //铂热电阻	通过分压电阻计算所得
int C ;                         //测量温度	分压计算
int XC;                         //目标温度

byte ms = 0;                    //模式切换
volatile byte EN = 0;           //模式使能
byte EN_tmp = 0;                //模式使能K4_Ring

void noTone(uint8_t pin, uint8_t channel = BUZZER_CHANNEL)
{
  ledcDetachPin(pin);
  ledcWrite(channel, 0);
}
void tone(uint8_t pin, note_t note, uint8_t octave, unsigned long duration, uint8_t channel = BUZZER_CHANNEL)
{
  if (ledcRead(channel)) {
    log_e("Tone channel %d is already in use", channel);
    return;
  }
  ledcAttachPin(pin, channel);
  ledcWriteNote(channel, note, octave);
  if (duration) {
    vTaskDelay(1);
    delay(duration);
    noTone(pin, channel);
  }
}

void K4_Ring() {
  if (EN_tmp == 1) {
    if (!EN) {
      tone(BUZZER_PIN, NOTE_B, 4, 100);
      tone(BUZZER_PIN, NOTE_A, 4, 100);
      tone(BUZZER_PIN, NOTE_G, 4, 100);
    } else {
      tone(BUZZER_PIN, NOTE_C, 4, 100);
      tone(BUZZER_PIN, NOTE_D, 4, 100);
      tone(BUZZER_PIN, NOTE_E, 4, 100);
    }
    noTone(BUZZER_PIN);
  }
  EN_tmp  = 0;
}

bool AutoWifiConfig()
{

  //设置固定IP地址
  //IPAddress staticIP(192, 168, 5, 24); //ESP static ip
  //IPAddress gateway(192, 168, 5, 1);   //IP Address of your WiFi Router (Gateway)
  //IPAddress subnet(255, 255, 255, 0);  //Subnet mask
  //IPAddress dns(192, 168, 5, 1);  //DNS
  //WiFi.config(staticIP, gateway, subnet, dns);
  WiFi.begin(ssid, password); //Wifi接入到网络
  //如果觉得时间太长可改
  for (int i = 0; i < 20; i++)
  {
    int wstatus = WiFi.status();
    if (wstatus == WL_CONNECTED)
    {
      Serial.println("WIFI AutoConfig Success");
      Serial.print("LocalIP:");
      Serial.print(WiFi.localIP());
      Serial.print(" ,GateIP:");
      Serial.println(WiFi.gatewayIP());
      return true;
    }
    else
    {
      Serial.print("WIFI AutoConfig Waiting......");
      Serial.println(wstatus);
      delay(1000);
    }
  }
  Serial.println("WIFI AutoConfig Faild!" );
  return false;
}

int V25;
//MCU电源电压校准补偿
void MCU()
{
  double tmp;
  //int V25  = map(analogRead(adc),0,1023,0,5000);

  //analogSetPinAttenuation(adc, ADC_11db);
  for (int i = 0; i < 20; i++)
  {
    //V25 = map(analogRead(adc), 0, 4095, 0, 3300);
    tmp  = tmp + analogReadMilliVolts(adc);
    //tmp = tmp + analogRead(adc);
  }
  //V25 = (((tmp / 20) * 3.3) / 4095);
  V25 = tmp / 20;

  adc_OFF_set = V25 - TL25;

  if (V25 > TL25)
  {
    VCC = 3300 - (V25 - TL25);
  } else if (V25 < TL25) {
    VCC = 3300 + (TL25 - V25);
  } else {
    VCC = 3300;
  }
}



int interruptCounter = 0;
//模式使能中断触发
void zd()
{
  interruptCounter++;
  while (digitalRead(k4) == LOW) {
    Serial.println(interruptCounter );
    //digitalWrite(buzz,LOW);
  }

  //digitalWrite(buzz,HIGH);
  EN = !EN;
  EN_tmp  = 1;
}

//模式切换检测
void kms()
{
  if (digitalRead(k3) == LOW)
  {
    ms++;
    while (digitalRead(k3) == LOW) {
      //digitalWrite(buzz,LOW);
      switch (ms)
      {
        case 0:
          tone(BUZZER_PIN, NOTE_C, 4, 100);
          break;
        case 1:
          tone(BUZZER_PIN, NOTE_D, 4, 100);
          break;
        case 2:
          tone(BUZZER_PIN, NOTE_E, 4, 100);
          break;
        case 3:
          tone(BUZZER_PIN, NOTE_F, 4, 100);
          break;
        case 4:
          tone(BUZZER_PIN, NOTE_G, 4, 100);
          break;
        case 5:
          tone(BUZZER_PIN, NOTE_A, 4, 100);
          break;
        default:
          tone(BUZZER_PIN, NOTE_B, 4, 100);
          break;
      }
    }

    //digitalWrite(buzz,HIGH);
  }
}

//PT1000分度表
int PT1000[] = {
  1000, 1019, 1039, 1058, 1078, 1097, 1117, 1036, 1155, 1175,   //0--45
  1194, 1213, 1232, 1252, 1271, 1290, 1309, 1328, 1347, 1366,   //50--95
  1385, 1404, 1422, 1438, 1460, 1479, 1498, 1517, 1536, 1555,   //100--145
  1573, 1592, 1610, 1629, 1647, 1666, 1684, 1703, 1722, 1740,   //150--195
  1758, 1777, 1795, 1814, 1832, 1850, 1868, 1886, 1904, 1923,   //200--245
  1941, 1959, 1977, 1995, 2013, 2031, 2049, 2067, 2085, 2103,   //250--295
  2120, 2138, 2156, 2174, 2191, 2209, 2227, 2245, 2262, 2280,   //300--345
  2297                                                          //350
};

//传入阻值查表并返回温度
float PT(float a)
{
  float c1 = 0;
  int jd = 50;         //精度0.1
  for (byte i = 0; i < sizeof(PT1000); i++)
  {
    if (PT1000[i] > a)
    {
      float c2 = float((PT1000[i] - PT1000[i - 1])) / jd ; //计算温度区间差值
      float c3 = PT1000[i - 1];                        //取最小温度分度
      for (int b = 0 ; b < jd; b++)
      {
        c3 = c3 + c2;
        if (c3 > a)
        { //循环比较温度
          c1 = float(i - 1) * 10 / 2 + float(b - 1) / 10; //整数+小数温度
          break;
        }
      }
      break;
    } else if (PT1000[i] == a) {
      c1 = float(i) * 10 / 2;
      break;
    }
  }
  return c1;
}

//测热电阻温度
void WD()
{
  int CC = 0;
  int swap = 0, MAX_Vr = 0, MIN_Vr = 0;     //使用中间变量swap、最大电阻MAX_R和最小电阻MIN_R，用以消除ADC偏离的值
  int r;

  //analogSetPinAttenuation(pr, ADC_11db);
  for (int i = 0; i < 20; i++)
  {
    //float r = map(analogRead(pr), 0, 4095, 0, VCC); //测电阻
    r = analogReadMilliVolts(pr);  //测电阻电压

    R = (2000 * r) / (5000 - r); //V=(R1+R2)/R2*V2 R2=(R1*r)/(5000-r)

    if (999 < R && R < 2300)
    {
      swap = PT(R);
      swap > MAX_Vr ? MAX_Vr = swap : 0;
      swap < MIN_Vr ? MIN_Vr = swap : 0;
      C = swap;
    } else {
      C = 0 ;
      R = 0 ;
      serial.printf("温度检测出错，请检查PT电阻及相关线路！\n");
    }
    CC = CC + C;
  }
  C = (CC - MAX_Vr - MIM_Vr) / 18;
}
float ADCa;
//测电压 电流 功率
void CL()
{
  MCU();
  double VV = 0;
  //analogSetPinAttenuation(dy, ADC_6db);
  for (int i = 0; i < 20; i++)
  {
    //float v  = map(analogRead(dy), 0, 4095, 0, VCC);
    //V = v * 6 / 1000.0;   //V=(R1+R2)/R2*V2*1000
    double v  = (analogReadMilliVolts(dy)+338) / 1000.0;        //这里为何加上338？
    //double v = analogRead(dy);
    //v = ((v * VCC) / 4095000);
    VV = VV + v + ((v * 100000) / 12800);                       //这里在计算什么？
  }
  V = (VV / 20);

  //float a  = map(analogRead(dl),0,4095,0,VCC);
  ADCa  = analogReadMilliVolts(dl);
  //a = a-(adc_OFF_set/2);
  A = abs(ADCa - 676) / 101 / 2; //电流 = 放大电压/放大倍数/采样电阻值mR

  W = V * A;
  if ( A > 0.1) {
    Z = V / A;
  } else {
    Z = 0;
  }
}


//传递引用 最小值 最大值区间加减
void key(int &a, int mi, int mx)
{
  if (digitalRead(k1) !=  digitalRead(k2))
  {
    long int t =  millis();
    if (digitalRead(k1) == LOW)
    {
      a++;
      while (digitalRead(k1) == LOW)
      {
        if (millis() - t > 500)
        {
          a = a + 2;
        }
        if (a > mx) {
          a = mx ;
        }
        oled4();
      }
    } else {
      a--;
      while (digitalRead(k2) == LOW)
      {
        if (millis() - t > 500)
        {
          a = a - 2;
        }
        if (a < mi) {
          a = mi ;
        }
        oled4();
      }
    }
    EEPROM.put(0, a);  //写入EEPROM数据
  }
}


void oled0()
{
  display.clearDisplay();                //清理1306屏幕，准备显示：
  display.setTextSize(2);                //设置字体大小，正比
  display.setTextColor(WHITE);           //设置字体颜色

  display.setCursor( 0 , 0 );
  display.print(C);
  display.setCursor( 59, 0 );
  display.print("C");

  display.setCursor( 0 , 18);
  display.print(V);
  display.setCursor( 59, 18);
  display.print("V");

  display.setTextSize(1);

  display.setCursor(90, 0 );
  display.print("1 PCB");

  display.setCursor(90, 8 );
  display.print("2 PCB");

  display.setCursor(90, 17);
  display.print("3 PCB");

  display.setCursor(90, 25);
  display.print("4 JRT");

  display.setCursor(80, 40);
  display.print(ADCa);

  display.setCursor(0, 50);
  display.print(adc_OFF_set);
  display.setCursor(80, 50);
  display.print(V25);

  display.display();                     //把缓存都显示
}




void oledPCB(byte a)
{
  display.clearDisplay();                //清理屏幕
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print(220);

  display.setCursor(0, 8);
  display.print(160);

  display.setCursor(0, 17);
  display.print(80);

  display.setCursor(0, 25);
  display.print(0);

  display.drawLine(18, 0, 18, 31, WHITE);
  display.drawLine(18, 31, 128, 31, WHITE);

  switch (a)
  {
    case 1 : display.drawLine(18, 31, 36, 13,  WHITE);
      display.drawLine(36, 13, 88, 13,  WHITE);
      display.drawLine(88, 13, 106, 31, WHITE);
      break;

    case 2 : display.drawLine(18, 31, 31, 18,  WHITE);
      display.drawLine(31, 18, 66, 18,  WHITE);
      display.drawLine(66, 18, 80, 10,  WHITE);
      display.drawLine(80, 10, 98, 10,  WHITE);
      display.drawLine(98, 10, 128, 31, WHITE);
      break;

    case 3 : display.drawLine(18, 31, 36, 13,  WHITE);
      display.drawLine(36, 13, 58, 13,  WHITE);
      display.drawLine(58, 13, 88, 0,  WHITE);
      display.drawLine(88, 0, 97, 0,  WHITE);
      display.drawLine(97, 0, 128, 31, WHITE);
      break;

    default:
      break;
  }

  display.setCursor(0, 50);
  if (EN) {
    display.print("Burning...!!!");
  }
  display.display();
}


void oled4()
{
  display.clearDisplay();                //清理1306屏幕，准备显示：
  display.setTextSize(2);                //设置字体大小，正比
  display.setTextColor(WHITE);           //设置字体颜色

  display.setCursor( 0 , 0 );
  display.print(XC);
  display.setCursor( 55, 0 );
  display.print("C");

  display.setCursor( 0 , 18);
  display.print(C);
  display.setCursor( 55, 18);
  display.print("C");

  display.setTextSize(1);

  display.setCursor( 84, 0 );
  display.print(V);
  display.setCursor(120, 0 );
  display.print("V");

  display.setCursor( 84, 8 );
  display.print(A);
  display.setCursor(120, 8 );
  display.print("A");

  display.setCursor( 84, 17);
  display.print(W);
  display.setCursor(120, 17);
  display.print("W");

  display.setCursor( 84, 25);
  display.print(Z);
  display.setCursor(120, 25);
  display.print("R");

  display.setCursor(0, 40);
  display.print(ADCa);

  display.setCursor(0, 50);
  if (EN) {
    display.print("Burning...!!!");
  }

  display.display();                     //把缓存都显示
}

void oled5()
{
  display.clearDisplay();                //清理1306屏幕，准备显示：
  display.setTextSize(2);                //设置字体大小，正比
  display.setTextColor(WHITE);           //设置字体颜色

  display.setCursor( 0 , 0);
  display.print(V);
  display.setCursor( 55, 0);
  display.print("V");

  display.setCursor( 0 , 18);
  display.print(C);
  display.setCursor( 55, 18);
  display.print("C");

  display.setTextSize(1);

  display.setCursor( 84, 0 );
  display.print(V);
  display.setCursor(120, 0 );
  display.print("V");

  display.setCursor( 84, 8 );
  display.print(A);
  display.setCursor(120, 8 );
  display.print("A");

  display.setCursor( 84, 17);
  display.print(W);
  display.setCursor(120, 17);
  display.print("W");

  display.setCursor( 84, 25);
  display.print(Z);
  display.setCursor(120, 25);
  display.print("R");

  display.setCursor( 0, 40 );
  display.print(WiFi.softAPIP());

  display.display();                     //把缓存都显示
}

//MOS驱动   目标温度
void MOS(int a)
{
  WD();
  XC = a;
  if (a >= C)
  {
    digitalWrite(mos1, HIGH);
  } else {
    digitalWrite(mos1, LOW);
  }
  CL();
  oled4();
}


//回流焊模式1   0-110-160-0
void PCB1()
{
  long int i = millis();
  while (C < 110 && millis() - i < 20000 && EN == 1 ) {
    MOS(110);
  }
  i = millis();
  while (millis() - i < 20000 && EN == 1 ) {
    MOS(110);  //20S预热区
  }
  i = millis();
  while (C < 160 && millis() - i < 30000 && EN == 1) {
    MOS(160);
  }
  i = millis();
  while (millis() - i < 60000 && EN == 1) {
    MOS(160);  //60S回流区
  }

  while (C > 50 && EN == 1) {
    MOS(0);  //冷却区
  }
  fmq(2000);                                          //完成
  if (EN == 1) {
    ms = 0;
  }
  EEPROM.get(0, XC);
}


//回流焊模式2   0-100-140-180-0
void PCB2()
{
  long int i = millis();
  while (C < 100 && millis() - i < 20000 && EN == 1) {
    MOS(100);
  }
  i = millis();
  while (millis() - i < 10000  && EN == 1) {
    MOS(100); //10S预热区
  }

  i = millis();
  while (C < 140 && millis() - i < 30000 && EN == 1) {
    MOS(140);
  }
  i = millis();
  while (millis() - i < 30000 && EN == 1) {
    MOS(140); //30S挥发区
  }

  i = millis();
  while (C < 180 && millis() - i < 60000 && EN == 1) {
    MOS(180);
  }
  i = millis();
  while (millis() - i < 5000 && EN == 1) {
    MOS(180); //5S回流区
  }

  while (C > 50 && EN == 1) {
    MOS(0);  //冷却区
  }
  fmq(2000);                                        //完成
  if (EN == 1) {
    ms = 0;
  }
  EEPROM.get(0, XC);
}



//回流焊模式3   0-100-160-220-0
void PCB3()
{
  long int i = millis();
  while (C < 100 && millis() - i < 20000 && EN == 1) {
    MOS(100);
  }
  i = millis();
  while (millis() - i < 10000 && EN == 1) {
    MOS(100); //10S预热区
  }

  i = millis();
  while (C < 160 && millis() - i < 30000 && EN == 1) {
    MOS(160);
  }
  i = millis();
  while (millis() - i < 30000 && EN == 1) {
    MOS(160); //30S挥发区
  }

  i = millis();
  while (C < 220 && millis() - i < 90000 && EN == 1) {
    MOS(220);
  }
  i = millis();
  while (millis() - i < 5000 && EN == 1) {
    MOS(220);  //5S回流区
  }

  while (C > 50 && EN == 1) {
    MOS(0);  //冷却区
  }
  fmq(2000);                                         //完成
  if (EN == 1) {
    ms = 0;
  }
  EEPROM.get(0, XC);
}

//加热台模式4
void JRT4()
{
  if (XC >= C && EN == 1)
  {
    digitalWrite(mos1, HIGH);
  } else {
    digitalWrite(mos1, LOW);
  }
  CL();
}



//蜂鸣器 延时
void fmq(int a)
{
  if (EN == 1)
  {
    tone(BUZZER_PIN, NOTE_C, 4, a);
  }
  digitalWrite(mos1, LOW);
}


void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);

  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  Serial.printf("Chip ID: %d\r\n", chipId);

  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(ESP.getEfuseMac() >> 32)); //print High 2 bytes
  Serial.printf("%08X\r\n", (uint32_t)ESP.getEfuseMac()); //print Low 4bytes.

  Serial.printf("Chip model = %s Rev %d\r\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores CpuFreqMHz = %u\r\n", ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("get Cycle Count = %u\r\n", ESP.getCycleCount());
  Serial.printf("SDK version:%s\r\n", ESP.getSdkVersion());  //获取IDF版本

  //获取片内内存  Internal RAM
  Serial.printf("Total heap size = %u\t", ESP.getHeapSize());
  Serial.printf("Available heap = %u\r\n", ESP.getFreeHeap());
  Serial.printf("Lowest level of free heap since boot = %u\r\n", ESP.getMinFreeHeap());
  Serial.printf("Largest block of heap that can be allocated at once = %u\r\n", ESP.getMaxAllocHeap());

  //SPI RAM
  Serial.printf("Total Psram size = %u\t", ESP.getPsramSize());
  Serial.printf("Available Psram = %u\r\n", ESP.getFreePsram());
  Serial.printf("Lowest level of free Psram since boot = %u\r\n", ESP.getMinFreePsram());
  Serial.printf("Largest block of Psram that can be allocated at once = %u\r\n", ESP.getMinFreePsram());

  pinMode(dy, INPUT);
  pinMode(dl, INPUT);
  pinMode(k1, INPUT_PULLUP);
  pinMode(k2, INPUT_PULLUP);
  pinMode(k3, INPUT_PULLUP);
  pinMode(k4, INPUT_PULLUP);
  pinMode(mos1, OUTPUT);
  //pinMode(buzz,OUTPUT);
  digitalWrite(mos1, LOW);
  //digitalWrite(buzz,HIGH);

  tone(BUZZER_PIN, NOTE_C, 4, 150);
  tone(BUZZER_PIN, NOTE_D, 4, 150);
  tone(BUZZER_PIN, NOTE_E, 4, 150);
  tone(BUZZER_PIN, NOTE_F, 4, 150);
  tone(BUZZER_PIN, NOTE_G, 4, 150);
  tone(BUZZER_PIN, NOTE_A, 4, 150);
  tone(BUZZER_PIN, NOTE_B, 4, 150);
  tone(BUZZER_PIN, NOTE_MAX, 4, 150);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); //初始化I2C地址0X3C

  attachInterrupt( k4, zd , FALLING);

  EEPROM.get(0, XC);    //读取EEPROM数据

  //wifi初始化
  WiFi.mode(WIFI_AP);
  while (!WiFi.softAP(ssid, password)) {}; //启动AP
  Serial.println("AP启动成功");
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  byte mac[6];
  WiFi.macAddress(mac);
  WiFi.setHostname(ServerName);
  Serial.printf("macAddress 0x%02X:0x%02X:0x%02X:0x%02X:0x%02X:0x%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  ArduinoOTA.setHostname(ServerName);
  //以下是启动OTA，可以通过WiFi刷新固件
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void loop()
{
  ArduinoOTA.handle();
  kms();
  WD();
  CL();
  K4_Ring();
  switch (ms)
  {
    case 0 : oled0(); EN = 0; digitalWrite(mos1, LOW);
      WiFi.mode(WIFI_OFF);
      break;

    case 1 : oledPCB(1);
      WiFi.mode(WIFI_OFF);
      if (EN == 1) {
        PCB1();
      }
      break;

    case 2 : oledPCB(2);
      WiFi.mode(WIFI_OFF);
      if (EN == 1) {
        PCB2();
      }
      break;

    case 3 : oledPCB(3);
      WiFi.mode(WIFI_OFF);
      if (EN == 1) {
        PCB3();
      }
      break;

    case 4 : JRT4();
      WiFi.mode(WIFI_OFF);
      key(XC, 0, 300);
      vTaskDelay(1);
      oled4();
      break;

    case 5 : oled5(); EN = 0; digitalWrite(mos1, LOW);
      WiFi.mode(WIFI_AP);
      while (!WiFi.softAP(ssid, password)) {}; //启动AP
      break;

    default: ms = 0; EN = 0;
      WiFi.mode(WIFI_OFF);
      break;
  }
}
