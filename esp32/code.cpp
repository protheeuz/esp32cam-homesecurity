/*
  Mathtech Digital Studio
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <ESP32Servo.h>
//#include <Servo.h>


#include "Arduino.h"
#include "FS.h"               
#include "SD_MMC.h"            
#include "driver/rtc_io.h"

const int hour_to_flash = 20; // jam untuk mengaktifkan autoFlash
const int hour_till_flash = 6; 

#include <TimeLib.h>
bool checkHour = false; // pengecekan jam untuk dapat mengaktifkan autoFlash
unsigned long offset_days = 3;    // 3 days
unsigned long t_unix_date1, t_unix_date2;


int armed = true;

const String photo_str = "📷Photo📷";
const String state_str = "📜Status📜";
const String arm_str = "🚨Pantauan🔥";
const String disarm_str = "🚨Dispantauan📵";
const String ledon_str = "💡LED-Hidup🔥";
const String ledoff_str = "💡LED-MATI📵";
const String vibs_str = "🚨Getaran AKTIF🔥";
const String disvibs_str = "🚨Getaran MATI📵";
const String sound_str = "🚨Suara AKTIF🔥";
const String dissound_str = "🚨Suara MATI📵";
const String servo_str = "🤖Buka Gerbang🤖";
const String disservo_str = "📵Tutup Gerbang📵";

const String buzzer_str = "🤖Buzzer🤖";
const String autoflash_str = "💡AutoFlash🤖";

const String flashOn_str = "📷Flash AKTIF💡";
const String flashOff_str = "📷Flash MATI📵";

const String keyboardJson = "[[\"" + photo_str + "\", \"" + state_str + "\"],[\"" + flashOn_str + "\", \" " + flashOff_str + "\"],[\"" + arm_str + "\", \" " + disarm_str + "\"],[\"" + ledon_str + "\", \"" + ledoff_str + "\"], [\"" + vibs_str + "\", \"" + disvibs_str + "\"], [\"" + sound_str + "\", \"" + dissound_str + "\"], [\"" + servo_str + "\", \"" + disservo_str + "\"], [\"" + autoflash_str + "\"],[\"" + buzzer_str + "\"] ]";




const char* ssid = "WIFI SSID";
const char* password = "WIFI PASSWORD";

String chatId = "YOUR_CHAT_ID";

// Initialize Telegram BOT
String BOTtoken = "YOUR_TELEGRAM_BOT_TOKEN"; // bot token

bool sendPhoto = false;

bool flashEnabled = false;


WiFiClientSecure clientTCP;

UniversalTelegramBot bot(BOTtoken, clientTCP);

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22



#define FLASH_LED_PIN 4
#define VIBRATION_SENSOR_PIN GPIO_NUM_12
#define SOUND_SENSOR_PIN GPIO_NUM_14
#define BUZZER_PIN GPIO_NUM_15
#define SERVO_PIN GPIO_NUM_2

Servo gateServo;


bool flashState = LOW;

// Motion Sensor (PIR)
bool motionDetected = false;

// Sound Sensor (KY-037)
bool soundDetected = false;

// Vibration Sensor (SW-420)
bool vibsDetected = false;

// buzzer
bool buzzDetected = false;



int botRequestDelay = 1000;   // waktu rata-rata antara pesan pemindaian
long lastTimeBotRan;     // terakhir kali pemindaian pesan selesai

void handleNewMessages(int numNewMessages);
String sendPhotoTelegram();
void toggleVibration(bool enable);
void toggleSound(bool enable);
void toggleBuzzer(bool enable);
void toggleServo(bool open);


// Indikasi ketika gerakan terdeteksi
static void IRAM_ATTR detectsMovement(void * arg) {
  //Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

// Indikasi ketika suara terdeteksi
static void IRAM_ATTR detectsSound(void * arg) {
  soundDetected = true;
 }

// Indikasi ketika geratan terdeteksi
static void IRAM_ATTR detectsVibration(void * arg) {
  vibsDetected = true;
}

// Indikasi buzzer
 static void IRAM_ATTR detectsBuzzer(void * arg) {
  buzzDetected = true;
 }

// Indikasi servo
  static void IRAM_ATTR detectsServo(void * arg) {
    gateServo.write(180);
    delay(1000);
    gateServo.write(0);
  }

void setup() {
  checkHour = false;
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);
  rtc_gpio_hold_dis(GPIO_NUM_4); // MIO



  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Terkoneksi kepada ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  WiFi.hostname("MathtechCamera");
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  

  //inisialisasi spesifikasi tertinggi untuk alokasi buffers
  if (psramFound()) {
    config.frame_size =  FRAMESIZE_VGA;//FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  // 0-63 itu kualitas gambar, bebas pilih
    config.fb_count = 4;//2;
  } else {
    config.frame_size = FRAMESIZE_VGA;//FRAMESIZE_UXGA;
    config.jpeg_quality = 10;//12;  // 0-63 itu kualitas gambar, bebas pilih
    config.fb_count = 2;//1;
  }

  // init kamera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

 
  //sensor_t * s = esp_camera_sensor_get();
  //s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);  //up the blightness just a bit
    s->set_saturation(s, -2); //lower the saturation
    s->set_sharpness(s, 2);   //Up THe sharpness
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_VGA);



  // PIR Motion Sensor mode INPUT_PULLUP
  //err = gpio_install_isr_service(0);
  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);
  if (err != ESP_OK) {
    Serial.printf("handler add failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
/* -------------------------------------------------------------- */
  // VIBRATION Sensor mode INPUT_PULLUP
  err = gpio_isr_handler_add(GPIO_NUM_12, &detectsVibration, (void *) 12);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(GPIO_NUM_12, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
/* -------------------------------------------------------------- */
  // SOUND Sensor mode INPUT_PULLUP
  err = gpio_isr_handler_add(GPIO_NUM_14, &detectsSound, (void *) 14);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(GPIO_NUM_14, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
/* -------------------------------------------------------------- */
  // BUZZER sensor mode INPUT_PULLUP
  err = gpio_isr_handler_add(GPIO_NUM_15, &detectsBuzzer, (void *) 15);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type (GPIO_NUM_15, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
/* -------------------------------------------------------------- */
  // SERVO sensor mode INPUT_PULLUP
  err = gpio_isr_handler_add(SERVO_PIN, &detectsServo, (void *) SERVO_PIN);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(SERVO_PIN, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }

  // SERVO setup
  gateServo.attach(SERVO_PIN);
 
}

void toggleVibration(bool enable) {
  
}

void toggleSound(bool enable) {
  
}

void toggleBuzzer(bool enable) {
  
}

void toggleServo(bool open) {
  if (open) {
    gateServo.write(180);
  } else {
    gateServo.write(0);
  }
}

void loop() {
  if (sendPhoto) {
    Serial.println("Preparing photo");
    sendPhotoTelegram();
    sendPhoto = false;
  }

  if (armed && motionDetected) {
    bot.sendMessage(chatId, "Gerakan terdeteksi!!", "");
    Serial.println("Gerakan terdeteksi");
    sendPhotoTelegram();
    motionDetected = false;
  }

  if (armed && soundDetected) {
    bot.sendMessage(chatId, "Suara terdeteksi!!", "");
    Serial.println("Suara terdeteksi");
    soundDetected = false;
  }

  if (armed && vibsDetected) {
    bot.sendMessage(chatId, "Getaran terdeteksi!!", "");
    Serial.println("Getaran terdeteksi");
    vibsDetected = false;
  }

  if (armed && buzzDetected) {
    bot.sendMessage(chatId, "Buzzer aktif!!", "");
    Serial.println("Buzzer aktif");
    buzzDetected = false;
  }


  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

}

String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  if (flashEnabled) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
  }

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");

    String head = "--Mathtech\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatId + "\r\n--Mathtech\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Mathtech--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=LucasMac");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }

  if (flashEnabled) {
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
  }
  return getBody;
}

void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {


    // chat id user request
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != chatId) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // print pesan yang diterika
    String text = bot.messages[i].text;
    Serial.println(text);

    telegramMessage message = bot.messages[i];
    int now_hour = getHourFromTelegram(message); // mendapatkan jam
    if ((checkHour) && (now_hour >= hour_to_flash &&  now_hour <= hour_till_flash)) {
      flashEnabled = true;  
    }
    
    
    String from_name = bot.messages[i].from_name;
    const String welcome = "Halo, " + from_name + ".  Selamat datang di command centre UNSADA SECURITY Bot. Silahkan pilih opsi dibawah untuk menjalankan perintah.\n";
    if (text == photo_str) {
      sendPhoto = true;
      Serial.println("New photo  request");
    }
    else if (text == ledon_str) {
      bot.sendMessageWithReplyKeyboard(chat_id, "Status LED diatur ke AKTIF", "", keyboardJson, true);
      //bot.sendMessage(chat_id, "LED state set to ON", "");
      digitalWrite(FLASH_LED_PIN, HIGH);
    }
    else if (text == ledoff_str) {
      bot.sendMessageWithReplyKeyboard(chat_id, "Status LED diatur ke TIDAK AKTIF", "", keyboardJson, true);
      //bot.sendMessage(chat_id, "LED state set to OFF", "");
      digitalWrite(FLASH_LED_PIN, LOW);
    }
    else if (text == arm_str) {
      armed = true;
      bot.sendMessageWithReplyKeyboard(chat_id, "Sistem Keamanan AKTIF", "", keyboardJson, true);
      //bot.sendMessage(chat_id, "Security System is ON", "");
    }
    else if (text == disarm_str) {
      armed = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Sistem Keamanan TIDAK AKTIF", "", keyboardJson, true);
      //bot.sendMessage(chat_id, "Security System is OFF", "");
    }
    else if (text == autoflash_str) {
      checkHour = !checkHour;
      if(checkHour){
        bot.sendMessageWithReplyKeyboard(chat_id, "Flash akan diaktifkan secara otomatis setelah pukul 20:00", "", keyboardJson, true);
        //bot.sendMessage(chat_id, "Flash will be automatically enabled after 20:00", "");
      }else{
        bot.sendMessageWithReplyKeyboard(chat_id, "Flash otomatis dinonaktifkan", "", keyboardJson, true);
        //bot.sendMessage(chat_id, "Automatic flash disabled", "");
      }
      flashEnabled = false;
    }
    else if (text == flashOn_str) {
      flashEnabled = true;
      bot.sendMessageWithReplyKeyboard(chat_id, "Flash diaktifkan untuk foto", "", keyboardJson, true);
      //bot.sendMessage(chat_id, "Flash enabled for photos", "");
    }
    else if (text == flashOff_str) {
      flashEnabled = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Flash dinonaktifkan untuk foto", "", keyboardJson, true);
      //bot.sendMessage(chat_id, "Flash disabled for photos", "");
    }
    else if (text == vibs_str) {
      armed = false;
//      flashEnabled = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Sensor getaran diaktifkan", "", keyboardJson, true);
      toggleVibration(true);
    }
    else if (text == disvibs_str) {
      armed = false;
//      flashEnabled = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Sensor getaran dinonaktifkan", "", keyboardJson, true);
      toggleVibration(false);
    }
    else if (text == sound_str) {
      armed = false;
//      flashEnabled = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Sensor suara diaktifkan", "", keyboardJson, true);
      toggleSound(true);
    }
    else if (text == dissound_str) {
      armed = false;
//      flashEnabled = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Sensor suara dinonaktifkan", "", keyboardJson, true);
      toggleSound(false);
    }
    else if (text == buzzer_str) {
      armed = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Buzzer aktif", "", keyboardJson, true);
      toggleBuzzer(true);
    }
    else if (text == servo_str) {
      Serial.println("Menerima perintah BUKA PINTU");
      gateServo.write(180);
      armed = false;
      bot.sendMessageWithReplyKeyboard(chat_id, "Pintu dibuka", "", keyboardJson, true);
      toggleServo(true);
    }
    else if (text == disservo_str) {
      Serial.println("Menerima perintah TUTUP PINTU");
      gateServo.write(0);
      bot.sendMessageWithReplyKeyboard(chat_id, "Pintu ditutup", "", keyboardJson, true);
      toggleServo(false);
    }

    
    else if (text == state_str) {
      String flashEnabledStr = "";
      String motionEnabledStr = "";
      String vibsEnabledStr = "";
      String soundEnabledStr = "";
      String buzzerEnabledStr = "";
      String servoEnabledStr = "";
      
      
      if (flashEnabled) flashEnabledStr = "✅";
      else  flashEnabledStr = "❌";
      
      if (armed)  motionEnabledStr = "✅";
      else  motionEnabledStr = "❌";

      if (armed)  vibsEnabledStr = "✅";
      else  vibsEnabledStr = "❌";

      if (armed)  soundEnabledStr = "✅";
      else  soundEnabledStr = "❌";

      if (armed) buzzerEnabledStr = "✅";
      else buzzerEnabledStr = "❌";

      if (armed) servoEnabledStr = "✅";
      else buzzerEnabledStr = "❌";

    

      String stat = "Terhubung dengan: " + String(ssid) +". Rssi: " + String(WiFi.RSSI()) + "\nip: " +  WiFi.localIP().toString() + "\nStatus PIR: " + motionEnabledStr + "\nStatus Flash: " + flashEnabledStr + "\nStatus Getaran: " + vibsEnabledStr + "\nStatus Suara: " + soundEnabledStr + "\nStatus Servo: " + servoEnabledStr + "\nStatus Buzzer: " + buzzerEnabledStr;
      bot.sendMessageWithReplyKeyboard(chat_id, stat, "", keyboardJson, true);
    }
    else {

      //String welcome = "Welcome, " + from_name + ".\n";
      //welcome += "Use the following commands to control your outputs.\n\n";
      //welcome += "/photo to take a picture \n";
      //welcome += "/arm to arm the security system \n";
      //welcome += "/disarm to disarm the security system \n";
      //welcome += "/ledon to turn flash light always on\n";
      //welcome += "/ledoff to turn flash light always off\n";
      //welcome += "/autoflash to turn on flash automatically at 20:00\n";
      //welcome += "/state to request current GPIO state and security system state \n";
      //bot.sendMessage(chat_id, welcome, "Markdown");

      
      
      bot.sendMessageWithReplyKeyboard(chat_id, welcome, "", keyboardJson, true);
    }
  }
}

int getHourFromTelegram(telegramMessage message) {
  String now_date = message.date; // mendapatkan jam & tgl hari ini

//  unsigned long offset_days = 3;    // 3 hari
  unsigned long t_unix_date1 = strtol(now_date.c_str(), NULL, 10);;
  int now_hour = hour(t_unix_date1);
  return now_hour + 7;  // jam hari ini + 7 GMT (jakarta)
}