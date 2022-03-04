#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
//#include "cactus_io_AM2315.h"
#include <avr/wdt.h>

#define SENSOR_GUNCELLEME_ARALIGI_MS 1000

#define PIN_MEMBRAN_SAG     8
#define PIN_MEMBRAN_YUKARI  9
#define PIN_MEMBRAN_ASAGI   10
#define PIN_MEMBRAN_SEC     11
#define PIN_MEMBRAN_SOL     12
#define PIN_MEMBRAN_BASLA   13

uint8_t membran_pin_listesi[]= {
 PIN_MEMBRAN_YUKARI,
 PIN_MEMBRAN_ASAGI,
 PIN_MEMBRAN_SAG,
 PIN_MEMBRAN_SOL,
 PIN_MEMBRAN_SEC,
 PIN_MEMBRAN_BASLA
};

#define PIN_PIEZO1 6
#define PIN_PIEZO2 6
#define PIN_FAN 7
#define PIN_DS18B20 1
//#define PIN_FAN_PWM1 9
//#define PIN_FAN_PWM2 10

//Modbus-SHT20
#define SSERIAL_RX_PIN 2
#define SSERIAL_TX_PIN 3
#define MAX485_DE 4
#define MAX485_RE_NEG 5

SoftwareSerial RS485Serial(SSERIAL_RX_PIN, SSERIAL_TX_PIN);
ModbusMaster node;

struct degisken_listesi {
  int8_t hum_min;
  int8_t hum_max;
//int8_t fan_speed;
  bool p_cont;
  int8_t fan_delay;
  bool makina_son_durum_calisiyo;
  uint8_t dak_calıs;
  uint8_t san_calıs;
  uint8_t dak_bekle;
  uint8_t san_bekle;
  uint16_t tekrar;
}degisken_listesi;

struct sensor_degerleri {
  float su_sicakligi;
  float nem;
  float sicaklik;
}sensor_degerleri;

typedef struct{
  uint32_t yukari : 1;
  uint32_t asagi : 1;
  uint32_t sag : 1;
  uint32_t sol : 1;
  uint32_t sec : 1;
  uint32_t basla : 1;
}membran_giris;

byte selsius[8] = {
  0x8,0xf4,0x8,0x43,0x4,0x4,0x43,0x0
};

byte nem[8] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B11111,
  B11111,
  B01110,
  B00000
};

byte kenar_sol_ust[8] = {
  B11110,
  B10000,
  B10000,
  B10000,
  B00000,
  B00000,
  B00000,
  B00000
};

byte kenar_sag_ust[8] = {
  B01111,
  B00001,
  B00001,
  B00001,
  B00000,
  B00000,
  B00000,
  B00000
};
byte kenar_sol_alt[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B10000,
  B10000,
  B10000,
  B11110
};
byte kenar_sag_alt[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00001,
  B00001,
  B00001,
  B01111
};

void saveConfig(){
  EEPROM.put(10,degisken_listesi);
}
void loadConfig(){
  EEPROM.get(10,degisken_listesi);
  bool badsetting = false;
  if(degisken_listesi.hum_min < 10 || degisken_listesi.hum_min > 99)
  {
    degisken_listesi.hum_min = 70;
    badsetting = true;
  }
  if(degisken_listesi.hum_max < 10 || degisken_listesi.hum_max > 99)
  {
    degisken_listesi.hum_max = 80;
    badsetting = true;
  }
//  if(degisken_listesi.fan_speed < 1 || degisken_listesi.fan_speed > 4)
//  {
//    degisken_listesi.fan_speed = 4;
//    badsetting = true;
//  }
  if(degisken_listesi.fan_delay < 5 || degisken_listesi.fan_delay > 15)
  {
    degisken_listesi.fan_delay = 5;
    badsetting = true;
  }
  if(badsetting){
    saveConfig();
  }
}

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1); 
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

LiquidCrystal_I2C ekran(0x27, 20, 4);
OneWire ds18b20_wire(PIN_DS18B20);
DallasTemperature ds18b20(&ds18b20_wire);   // tek basina olan probe
//AM2315 am2315;                              // sicaklik-nem bir arada

// global variables
bool manual_start_istendi = false;
uint8_t ekran_indexi = 0;

bool ekran_temizlenmeli=true;

void makinayi_baslat(){
    manual_start_istendi = true;
    ekran_indexi = 4;
    ekran_temizlenmeli=true;
    degisken_listesi.makina_son_durum_calisiyo = true;
    saveConfig();
}

void ekran_secim(membran_giris * giris){
  if(ekran_temizlenmeli){
    ekran_temizlenmeli=false;
    ekran.clear();  
  }
  if(giris->sag){
    ekran_indexi=7;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }
  ekran.setCursor(1,1);
  ekran.print("Calısma Modu Secimi");
  ekran.setCursor(2,1);
  ekran.print("Sensorlu | Zamanli");
}

void ekran0(membran_giris * giris){ // hoşgeldin ekranı
  if(ekran_temizlenmeli){
    ekran_temizlenmeli=false;
    ekran.clear();  
  }
  if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi = 3;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }
  ekran.setCursor(0, 0);
  ekran.write(byte(5));
  ekran.setCursor(19, 0);
  ekran.write(byte(6));
  ekran.setCursor(0, 3);
  ekran.write(byte(7));
  ekran.setCursor(19, 3);
  ekran.write(byte(8));

  ekran.setCursor(1,1);
  ekran.print("SICAKLIK");
  ekran.setCursor(1,2);
  ekran.print(sensor_degerleri.sicaklik);
  ekran.print((char)223);
  ekran.print("C");

  if (sensor_degerleri.sicaklik < 10) {
    ekran.setCursor(7, 2);
    ekran.print(" ");
  }
  if (sensor_degerleri.sicaklik < -0.00) {
    ekran.setCursor(7, 2);
    ekran.print("C");
  }
  if (sensor_degerleri.sicaklik < -10.00) {
    ekran.setCursor(8, 2);
    ekran.print("C");
  }  
  
  ekran.setCursor(16,1);
  ekran.print("NEM");  
  ekran.setCursor(13,2);
  ekran.print(sensor_degerleri.nem);
  ekran.print("%");
  
}

void ekran1(membran_giris * giris){ // minimum nem
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
    if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.hum_min++;
    degisken_listesi.hum_min = constrain(degisken_listesi.hum_min, 0 ,degisken_listesi.hum_max-1);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.hum_min--;
    degisken_listesi.hum_min = constrain(degisken_listesi.hum_min, 0 ,degisken_listesi.hum_max-1);
    saveConfig();
  }
    ekran.setCursor(0, 0);
    ekran.print("MiNiMUM NEM: ");
    ekran.print(degisken_listesi.hum_min);
    ekran.print("%");
    ekran.print("  ");
}

void ekran2(membran_giris * giris){ // maximum nem
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
    if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.hum_max++;
    degisken_listesi.hum_max = constrain(degisken_listesi.hum_max, degisken_listesi.hum_min+1 ,99);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.hum_max--;
    degisken_listesi.hum_max = constrain(degisken_listesi.hum_max, degisken_listesi.hum_min+1 ,99);
    saveConfig();
    }
    ekran.setCursor(0, 0);
    ekran.print("MAKSiMUM NEM: ");
    ekran.print(degisken_listesi.hum_max);
    ekran.print("%");
    ekran.print("  ");
}

void ekran3(membran_giris * giris){ // fan delay
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
  if(giris->sag){
    ekran_indexi = 0;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.fan_delay+=5;
    degisken_listesi.fan_delay = constrain(degisken_listesi.fan_delay, 5 ,15);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.fan_delay-=5;
    degisken_listesi.fan_delay = constrain(degisken_listesi.fan_delay, 5 ,15);
    saveConfig();
  }
    ekran.setCursor(0, 0);
    ekran.print("FAN GECiKME: ");
    ekran.print(degisken_listesi.fan_delay);
    ekran.print("sn.");
    ekran.print("  ");
}

void ekran4(membran_giris * giris){ // calisma ekrani
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
    ekran.setCursor(0, 0);
    ekran.print("NEM:");
    ekran.setCursor(5, 0);
    ekran.print(sensor_degerleri.nem);
    ekran.print("%");
    ekran.setCursor(0, 1);
    ekran.print("MiN: ");    
    ekran.print(degisken_listesi.hum_min);
    ekran.print("%");
    ekran.setCursor(8, 1);
    ekran.print(" | MAKS: ");
    ekran.print(degisken_listesi.hum_max);
    ekran.print("%");
    ekran.setCursor(0, 2);
    ekran.print("SICAKLIK:");
    ekran.setCursor(10, 2);
    ekran.print(sensor_degerleri.sicaklik);  
    ekran.print((char)223);
    ekran.print("C");
    if (sensor_degerleri.sicaklik < 10) {
    ekran.setCursor(16, 2);
    ekran.print(" ");
    }    
    if (sensor_degerleri.su_sicakligi != 12.3 ) {
      ekran.setCursor(0, 3);
      ekran.print("SU:");    
      ekran.setCursor(4, 3);
      ekran.print(sensor_degerleri.su_sicakligi);
      ekran.print((char)223);
      ekran.print("C");
        if (sensor_degerleri.su_sicakligi < 10) {
        ekran.setCursor(10, 2);
        ekran.print(" ");              
        }
  }
  if(giris->basla || giris->sec){
    manual_start_istendi = false;
    ekran_indexi = 0;
    ekran_temizlenmeli=true;
    degisken_listesi.makina_son_durum_calisiyo = false;
    saveConfig();
  }
}

bool ekran5_aktif=false;
void ekran5(membran_giris * giris){ // su ısı hata ekranı
  if(!ekran5_aktif){
     ekran.clear();  
     ekran.setCursor(0, 1);
     ekran.print("HATA: SU SICAKLIK"); 
     ekran.setCursor(11, 2);
     ekran.print(sensor_degerleri.su_sicakligi);
     ekran.print((char)223);
     ekran.print("C"); 
     ekran5_aktif = true;
  } 
  ekran5_aktif = true;
}

void ekran6(membran_giris * giris){
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
  if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi=0;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.dak_calıs++;
    degisken_listesi.dak_calıs = constrain(degisken_listesi.dak_calıs, 0 ,60);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.dak_calıs--;
    degisken_listesi.dak_calıs= constrain(degisken_listesi.dak_calıs, 0 ,60);
    saveConfig();
  }

  ekran.setCursor(0, 0);
  ekran.print("Dakika Saniye ");
  ekran.setCursor(1, 0);
  ekran.print(degisken_listesi.dak_calıs);
  ekran.setCursor(1, 3);
  ekran.print(degisken_listesi.san_calıs);
  ekran.setCursor(1, 6);
  ekran.print("calıs.");
}

void ekran7(membran_giris * giris){
  if(ekran_temizlenmeli){
   ekran_temizlenmeli=false;
   ekran.clear();  
  }
  if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.san_calıs++;
    degisken_listesi.san_calıs = constrain(degisken_listesi.san_calıs, 0 ,60);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.san_calıs--;
    degisken_listesi.san_calıs= constrain(degisken_listesi.san_calıs, 0 ,60);
    saveConfig();
  }

  ekran.setCursor(0, 0);
  ekran.print("Dakika Saniye ");
  ekran.setCursor(1, 0);
  ekran.print(degisken_listesi.dak_calıs);
  ekran.setCursor(1, 3);
  ekran.print(degisken_listesi.san_calıs);
  ekran.setCursor(1, 6);
  ekran.print("calıs.");
}

void ekran8(membran_giris * giris){
  if(ekran_temizlenmeli){
    ekran_temizlenmeli=false;
    ekran.clear();  
  }
  if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.dak_bekle++;
    degisken_listesi.dak_bekle = constrain(degisken_listesi.dak_bekle, 0 ,60);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.dak_bekle--;
    degisken_listesi.dak_bekle= constrain(degisken_listesi.dak_bekle, 0 ,60);
    saveConfig();
  }

  ekran.setCursor(0, 0);
  ekran.print("Dakika Saniye ");
  ekran.setCursor(1, 0);
  ekran.print(degisken_listesi.dak_bekle);
  ekran.setCursor(1, 3);
  ekran.print(degisken_listesi.san_bekle);
  ekran.setCursor(1, 6);
  ekran.print("bekle.");
}

void ekran9(membran_giris * giris){
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
    if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.san_bekle++;
    degisken_listesi.san_bekle = constrain(degisken_listesi.san_bekle, 0 ,60);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.san_bekle--;
    degisken_listesi.san_bekle= constrain(degisken_listesi.san_bekle, 0 ,60);
    saveConfig();
  }

  ekran.setCursor(0, 0);
  ekran.print("Dakika Saniye ");
  ekran.setCursor(1, 0);
  ekran.print(degisken_listesi.dak_bekle);
  ekran.setCursor(1, 3);
  ekran.print(degisken_listesi.san_bekle);
  ekran.setCursor(1, 6);
  ekran.print("bekle.");
}

void ekran10(membran_giris * giris){
   if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
    if(giris->sag){
    ekran_indexi++;
    ekran_temizlenmeli=true;
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    makinayi_baslat();
  }else if(giris->yukari){
    degisken_listesi.tekrar++;
    degisken_listesi.tekrar = constrain(degisken_listesi.tekrar, 0 ,999);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.tekrar--;
    degisken_listesi.tekrar= constrain(degisken_listesi.tekrar, 0 ,999);
    saveConfig();
  }

  ekran.setCursor(0, 0);
  ekran.print("Tekrar Sayisi");
  ekran.setCursor(1, 0);
  ekran.print(degisken_listesi.tekrar);
}

void ekran11(){
  if(ekran_temizlenmeli){
     ekran_temizlenmeli=false;
     ekran.clear();  
  }
  if(giris->sag){
    //void
  }else if(giris->sol){
    ekran_indexi--;
    ekran_temizlenmeli=true;
  }else if(giris->basla || giris->sec){
    for(int i; i<degisken_listesi.tekrar; i++){
     makinayi_baslat();
     delay(degisken_listesi.dak_calıs*60000);
     delay(degisken_listesi.san_calıs*1000);
     
      
    }
  }else if(giris->yukari){
    degisken_listesi.fan_gecikme=degisken_listesi.fan_gecikme+5;
    degisken_listesi.fan_gecikme = constrain(degisken_listesi.fan_delay, 5 ,15);
    saveConfig();
  }else if(giris->asagi){
    degisken_listesi.fan_gecikme=degisken_listesi.fan_gecikme-5;
    degisken_listesi.fan_gecikme= constrain(degisken_listesi.fan_delay, 5 ,15);
    saveConfig();
  }

  ekran.setCursor(0, 0);
  ekran.print("Fan Gecikme Suresi");
  ekran.setCursor(1, 0);
  ekran.print(degisken_listesi.fan_delay);
}

typedef void (*ekran_yapisi)(membran_giris * giris);
ekran_yapisi ekranlar[13]; 
void setup() {
  wdt_reset();
  wdt_disable();
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);   
  Serial.begin(9600);
  degisken_listesi = EEPROM.get(10, degisken_listesi);    // TODO : implement EEPROM reading
  ds18b20.begin();
  RS485Serial.begin(9600);
  node.begin(1, RS485Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
    
  Wire.setClock(400000); // Wire.setClock(400000)
  ekran.begin();
  ekran.createChar(5, kenar_sol_ust);
  ekran.createChar(6, kenar_sag_ust);
  ekran.createChar(7, kenar_sol_alt);
  ekran.createChar(8, kenar_sag_alt);
  
  //am2315.begin();
  loadConfig();
  for(uint8_t i = 0; i < sizeof(membran_pin_listesi); i++){
    pinMode(membran_pin_listesi[i],INPUT_PULLUP);
  }
  pinMode(PIN_PIEZO1, OUTPUT);
  pinMode(PIN_PIEZO2, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_PIEZO1, HIGH);
  digitalWrite(PIN_PIEZO2, HIGH);
  digitalWrite(PIN_FAN, HIGH);
  ekran.backlight();
  ekranlar[0] = ekran_secim;
  ekranlar[1] = ekran0;
  ekranlar[2] = ekran1;
  ekranlar[3] = ekran2;
  ekranlar[4] = ekran3;
  ekranlar[5] = ekran4;
  ekranlar[6] = ekran5
  ekranlar[7] = ekran6;
  ekranlar[8] = ekran7;
  ekranlar[9] = ekran8;
  ekranlar[10] = ekran9;
  ekranlar[11] = ekran10;
  ekranlar[12] = ekran11;
  
  ekran.setCursor(1, 1);
  if(degisken_listesi.makina_son_durum_calisiyo){
    makinayi_baslat();
  }
  ekran.print("HOS GELDiNiZ,");
  delay(2500);
  ekran.setCursor(1, 2);
  ekran.print("MAKiNE ACILIYOR.");
  delay(2500);
  wdt_enable(WDTO_4S);
}

membran_giris eski_tus;
membran_giris bos_tus = {0};
membran_giris membrani_oku(){

   membran_giris okunan_tus_bilgisi = {0};
   if(!digitalRead(PIN_MEMBRAN_YUKARI)){
    okunan_tus_bilgisi.yukari = 1;
   }else{
    okunan_tus_bilgisi.yukari = 0;
   }
   if(!digitalRead(PIN_MEMBRAN_ASAGI)){
    okunan_tus_bilgisi.asagi = 1;
   }else{
    okunan_tus_bilgisi.asagi = 0;
   }
   if(!digitalRead(PIN_MEMBRAN_SAG)){
    okunan_tus_bilgisi.sag = 1;
   }else{
    okunan_tus_bilgisi.sag = 0;
   }
   if(!digitalRead(PIN_MEMBRAN_SOL)){
    okunan_tus_bilgisi.sol = 1;
   }else{
    okunan_tus_bilgisi.sol = 0;
   }
   if(!digitalRead(PIN_MEMBRAN_BASLA)){
    okunan_tus_bilgisi.basla = 1;
   }else{
    okunan_tus_bilgisi.basla = 0;
   }
   if(!digitalRead(PIN_MEMBRAN_SEC)){
    okunan_tus_bilgisi.sec = 1;
   }else{
    okunan_tus_bilgisi.sec = 0;
   }
  if(memcmp(&eski_tus,&okunan_tus_bilgisi,sizeof(eski_tus))==0){
    return bos_tus;
  }
  memcpy(&eski_tus,&okunan_tus_bilgisi,sizeof(eski_tus));
  return okunan_tus_bilgisi;
}

uint32_t son_okuma_zamani;
void sensorleri_oku(){
  if(millis()-son_okuma_zamani > SENSOR_GUNCELLEME_ARALIGI_MS){
  ds18b20.requestTemperatures();
  sensor_degerleri.su_sicakligi = ds18b20.getTempCByIndex(0);
  Serial.println(sensor_degerleri.su_sicakligi);
  if(abs(sensor_degerleri.su_sicakligi) == 127){
    sensor_degerleri.su_sicakligi = 12.3;
  }
  
  //am2315.readSensor();
  uint8_t result;
  uint16_t data[2];
  result = node.readInputRegisters(1, 2);
  
  if (result == node.ku8MBSuccess)  {
    sensor_degerleri.sicaklik = node.getResponseBuffer(0)/10.0f;
    sensor_degerleri.nem = node.getResponseBuffer(1)/10.0f;    
  }
    Serial.print("Temperature: "); 
    Serial.println(node.getResponseBuffer(0)/10.0f);
    Serial.print("Humidity: "); 
    Serial.println(node.getResponseBuffer(1)/10.0f);
    Serial.println();   
    
  son_okuma_zamani = millis();
  }
}

uint32_t ekran_son_guncelleme;
bool ekranda_hata_gosterilmeli = false;
void ekrani_guncelle(membran_giris *giris){
  if(ekranda_hata_gosterilmeli){
    ekranlar[5](giris);
  }else{
    if(ekran5_aktif){
       ekran5_aktif = false;
       ekran.clear();
    }
    bool kullanici_girdi_yapti = memcmp(giris,&bos_tus,sizeof(bos_tus))!=0;
    if(kullanici_girdi_yapti){
      ekran_son_guncelleme = 0;
      ekranlar[ekran_indexi](giris);
    }
    if(!kullanici_girdi_yapti && ((millis()-ekran_son_guncelleme)>500)){ //475'ti
        ekran_son_guncelleme = millis();
        ekranlar[ekran_indexi](giris);
    }
  }
}

uint32_t fan_delay_baslangici;
bool last_state;
void kontrol(){
  bool su_sicakligi_istenilen_aralikta  = (sensor_degerleri.su_sicakligi > 5 && sensor_degerleri.su_sicakligi < 60);
  if(manual_start_istendi && su_sicakligi_istenilen_aralikta){
    if(sensor_degerleri.nem <= degisken_listesi.hum_min){
          digitalWrite(PIN_PIEZO1, LOW);
          digitalWrite(PIN_PIEZO2, LOW);
          digitalWrite(PIN_FAN, LOW);
          last_state=LOW;
    }
    if(sensor_degerleri.nem > degisken_listesi.hum_max){
          if(last_state==LOW){
            fan_delay_baslangici = millis();
          }
          digitalWrite(PIN_PIEZO1, HIGH);
          digitalWrite(PIN_PIEZO2, HIGH);
          last_state=HIGH;
          if(millis()-fan_delay_baslangici > (degisken_listesi.fan_delay*1000)){
            digitalWrite(PIN_FAN, HIGH);
          }
    }
        }else{
          if(last_state==LOW){
            fan_delay_baslangici = millis();
          }
          last_state=HIGH;
          digitalWrite(PIN_PIEZO1, HIGH);
          digitalWrite(PIN_PIEZO2, HIGH);
          if(millis()-fan_delay_baslangici > (degisken_listesi.fan_delay*1000)){
            digitalWrite(PIN_FAN, HIGH);
          }
  }
    if(sensor_degerleri.su_sicakligi <= 5 || sensor_degerleri.su_sicakligi >= 60){
      ekranda_hata_gosterilmeli = true;
    }else{
      ekranda_hata_gosterilmeli = false;
    }
}

void loop() {
  membran_giris membran_girdisi = membrani_oku();
  sensorleri_oku(); 
  ekrani_guncelle(&membran_girdisi);
  kontrol();
  wdt_reset(); 
  delay(10);
}
