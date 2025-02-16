/*  ============================================================================

     MrDIY.ca CAN Project modified by Jakkrit for Ford Ranger T5 & Merceses Benz S320 - 15 December 24

     APPLICATION: Gates opener

     Listen:     
                 - BENZ: the "Speak button" on the steering wheel pressed for more than 5 seconds
                 - FORD Ranger: cruise control button pressed for more than 5 seconds

     Action:     sends an MQTT garage door opener ( throught the ESPNow to Wi-Fi gateway)

  ============================================================================================== */

#define DEBUG_FLAG 1

#ifdef DEBUG_FLAG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// ------------- ESPNow  ---------------------

#include <ESP8266WiFi.h>
#include <espnow.h>

union sensor {
  unsigned  long long raw_ul;
  uint8_t d[8];
};

typedef struct esp_frame_t {
  int mesh_id;
  unsigned long can_id;
  byte len;
  uint8_t d[8];
};

esp_frame_t esp_frame;
sensor s;

// ------------- DBC ------------------------
/*
   BENZ S320
      BO_ 962 ID3C2VCLEFT_switchStatus: 8 VehicleBus
        SG_ VCLEFT_switchStatusIndex M : 0|2@1+ (1,0) [0|2] ""  Receiver
        SG_ VCLEFT_swcRightPressed m1 : 12|2@1+ (1,0) [0|3] ""  Receiver

   Ford Ranger
      BO_ 1265 CLU11: 4 CLU
        SG_ CF_Clu_CruiseSwMain : 3|1@1+ (1.0,0.0) [0.0|1.0] ""  EMS,LDWS_LKAS,SCC
*/

#define SPEAK_BUTTON_CAN_ID_BENZ          1014   // 0x3F6 (Decimal Value-1014)
#define CRUISE_STATE_BUTTON_CAN_ID_FORD   154  // 0x09A (Decimal Value-154) 

#define BTN_FORD_CRUISE_UNPRESSED 0
#define BTN_FORD_CRUISE_PRESSED   1

#define BTN_BENZ_SPEAK_UNPRESSED  0
#define BTN_BENZ_SPEAK_PRESSED    1

int hb_status;
bool btn_is_held_on = false;
unsigned long btn_press_started;
#define REQUIRED_HOLD_DURATION_IN_SECONDS  3

// --------- GARAGE OPENER ------------------

#define GARAGE_RELAY_PIN   5      // D1 (GPIO5) Relay module
#define LED   2      // D4 (GPIO2) Buildin LED
#define GARAGE_PRESS_DELAY 1000
#define GARAGE_PRESS_DELAY2 3000


//-------------------------------------------------------------------------------------------

void setup() {

#ifdef DEBUG_FLAG
  Serial.begin(115200);
  pinMode(GARAGE_RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(GARAGE_RELAY_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

#endif

  WiFi.mode(WIFI_STA);
  delay(10);
  debugln();
  debugln();
  debugln(F("=================================================================="));
  debugln(F("  CAN Bus Decoder System"));
  debugln(F("  GADGET: Gate opener"));
  debugln(F("  Modified by Jakkrit - 15 December 2024"));
  debugln(F("=================================================================="));
  debugln();
  debugln(" ESPNOW ......... INIT");

  while (esp_now_init() != 0) {}

  debugln(" ESPNOW ...... OK, I'm ready to received packages!!");
  debugln("");
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
  btn_is_held_on = false;
  btn_press_started = 0;
}

//-------------------------------------------------------------------------------------------

void loop() {
  if (btn_press_started > 0 && millis() - btn_press_started > REQUIRED_HOLD_DURATION_IN_SECONDS * 1000) {
    openGarageDoor();
    btn_press_started = 0;
    btn_is_held_on = false;
  }
}

//-------------------------------------------------------------------------------------------

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {

  memcpy(&esp_frame, incomingData, sizeof(esp_frame));

  if ( esp_frame.can_id == SPEAK_BUTTON_CAN_ID_BENZ) {
    processRaw(&esp_frame, &s, 12, 1);  // bit 13th need to be logic 1
    hb_status = (int) s.raw_ul;

    if ( btn_is_held_on == false && hb_status == BTN_BENZ_SPEAK_PRESSED) {
      btn_is_held_on = true;
      btn_press_started = millis();
    }

    if ( hb_status == BTN_BENZ_SPEAK_UNPRESSED ) {
      btn_is_held_on = false;
      btn_press_started = 0;
    }


  } else if ( esp_frame.can_id == CRUISE_STATE_BUTTON_CAN_ID_FORD) {

    //LED_ON_OFF();
    processRaw(&esp_frame, &s, 56, 1);  // bit 56th need to be logic 1, (Last bit)
    hb_status = (int) s.raw_ul;

    if ( btn_is_held_on == false && hb_status == BTN_FORD_CRUISE_PRESSED) {
      btn_is_held_on = true;
      btn_press_started = millis();
    }

    if ( hb_status == BTN_FORD_CRUISE_UNPRESSED ) {
      btn_is_held_on = false;
      btn_press_started = 0;
    }
  }
}

void processRaw(esp_frame_t *frame, sensor *s, int start_bit, int len) {
  for (int i = 0; i < 8 ; i++) s->d[i] = frame->d[i];
  s->raw_ul = s->raw_ul << (64 - start_bit - len)  >> (64 - len);
}

void openGarageDoor() {

  digitalWrite(LED_BUILTIN, LOW);
  debugln(F("Opening Garage"));
  digitalWrite(GARAGE_RELAY_PIN, HIGH);
  delay(GARAGE_PRESS_DELAY);
  digitalWrite(GARAGE_RELAY_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
}

void LED_ON_OFF() {
  //digitalWrite(LED_BUILTIN, LOW);
  //debugln(F("LED_ON_OFF"));
  digitalWrite(LED, LOW);
  delay(GARAGE_PRESS_DELAY2);
  digitalWrite(LED, HIGH);
  //digitalWrite(LED_BUILTIN, HIGH);
}