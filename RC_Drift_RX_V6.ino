// RC Drift RX V6.0 - ESP32-WROOM32, Arduino ESP32 3.3.x, 4MB Flash
// Compatible with TX V6 legacy 34-byte control + V6 30-byte accessory packets.
// Real BAC Mono sample-bank generated during GitHub build; synthesized fallback when absent.
// Attribution: Mark Oosting engine-audio MIT license; see third_party_LICENSE.txt.
// GPIO: Servo18 ESC19 IMU SDA21 SCL22 INT34 BAT35 Hood23
//       Engine DAC25 Rear DAC26 WS2812 DATA14 RPM A32 B33 Exhaust actuator13.
// IMPORTANT: GPIO13 one-wire reversible motor controller is DISABLED until hardware type verified.
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <esp_timer.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>
#if __has_include("EngineSamples.h")
#include "EngineSamples.h"
#define HAS_REAL_ENGINE_SAMPLES 1
#else
#define HAS_REAL_ENGINE_SAMPLES 0
#endif

static constexpr int SERVO_PIN=18, ESC_PIN=19, IMU_SDA=21, IMU_SCL=22, IMU_INT=34;
static constexpr int BATTERY_PIN=35, HOOD_PIN=23, FRONT_DAC_PIN=25, REAR_DAC_PIN=26;
static constexpr int STRIP_PIN=14, RPM_A_PIN=32, RPM_B_PIN=33, EXHAUST_PIN=13;
static constexpr int LED_COUNT=17, AUDIO_RATE=16000, ESC_NEUTRAL=1500;
static constexpr uint8_t RADIO_CHANNEL=6;
static constexpr uint32_t MAGIC_LEGACY=0xD4A52026UL, MAGIC_EXTRA=0xD6A52026UL;
static constexpr uint8_t LEGACY_VERSION=5, EXTRA_VERSION=6;
static constexpr uint32_t FAILSAFE_US=150000, EXTRA_TIMEOUT_MS=600;
// Command neutral after 150ms without a valid TX packet; telemetry remains diagnostic.
// Driver must understand one-wire, servo-style reversible input 1000-2000us.
#define EXHAUST_ACTUATOR_ENABLED 0
#define MOTOR_ENCODER_PPR_X4 40  // Change to your encoder's real quadrature counts/rev.

struct __attribute__((packed)) ControlPacket {
 uint32_t magic; uint8_t version,type,flags,mode; uint16_t sequence;
 int16_t steer,throttle; uint16_t gyroPot,speedPot,gyroSetting;
 uint16_t maxForward,maxReverse; uint8_t previewDir,actionCode;
 uint16_t previewValue,actionId; uint32_t echoTimestampUs;
};
struct __attribute__((packed)) TelemetryV5 {
 uint32_t magic; uint8_t version,type,linked,gyroOK; uint16_t rxPps,batteryMv;
 int8_t rxRssi; uint8_t armed,actionError; uint16_t actionAck;
 uint32_t echoTimestampUs; uint16_t servoLeftUs,servoCenterUs,servoRightUs;
};
struct __attribute__((packed)) AccessoryPacket {
 uint32_t magic; uint8_t version,type; uint16_t sequence;
 uint8_t gear,hoodOpen,hazard,leftSignal,rightSignal,hornToggle;
 uint8_t headlights,brakeLevel,engineVol,exhaustVol,gyroEnabled,gyroReverse;
 uint8_t gyroFilter,gyroDamping,tailAtIdle,steeringReverse;
 uint16_t rpmRequest,gyroGain; uint8_t lightBrightness,signalEnabled;
};
static_assert(sizeof(ControlPacket)==34,"TX control protocol size mismatch");
static_assert(sizeof(TelemetryV5)==27,"TX telemetry protocol size mismatch");
static_assert(sizeof(AccessoryPacket)==30,"TX accessory protocol size mismatch");

const uint8_t BROADCAST_MAC[6]={255,255,255,255,255,255};
portMUX_TYPE rxMux=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t rxAtUs=0, extraAtMs=0, packetCount=0;
volatile int8_t lastRSSI=-127;
ControlPacket rxControl={}; AccessoryPacket rxExtra={};
volatile bool haveControl=false,haveExtra=false;
uint32_t lastPackets=0; uint16_t packetsPerSec=0;
volatile bool sendingTelemetry=false; uint32_t sendStarted=0;
volatile uint32_t telemetryCallbacks=0,telemetrySendFailures=0,telemetryDeliveryFailures=0;
uint8_t txMac[6]={0};bool txMacKnown=false,txPeerRegistered=false;
uint8_t activeTxMac[6]={0};
uint32_t lastTXPeerAttempt=0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
void onSent(const esp_now_send_info_t*,esp_now_send_status_t status){
 telemetryCallbacks++;if(status!=ESP_NOW_SEND_SUCCESS)telemetryDeliveryFailures++;
 sendingTelemetry=false;
}
#else
void onSent(const uint8_t*,esp_now_send_status_t status){
 telemetryCallbacks++;if(status!=ESP_NOW_SEND_SUCCESS)telemetryDeliveryFailures++;
 sendingTelemetry=false;
}
#endif
void onReceive(const esp_now_recv_info_t *info,const uint8_t *data,int len){
 if(!info||!info->src_addr||!data)return;
 if(len==(int)sizeof(ControlPacket)){
  ControlPacket p;memcpy(&p,data,sizeof(p));
  if(p.magic!=MAGIC_LEGACY||p.version!=LEGACY_VERSION||p.type!=1||p.mode>3||
     abs((int)p.steer)>1000||abs((int)p.throttle)>1000||p.speedPot>1000||
     p.gyroPot>1000||p.gyroSetting>1000||p.maxForward>1000||p.maxReverse>1000)return;
  portENTER_CRITICAL(&rxMux);
  rxControl=p;rxAtUs=micros();haveControl=true;packetCount++;
  memcpy(activeTxMac,info->src_addr,6);txMacKnown=true;
  lastRSSI=info->rx_ctrl?info->rx_ctrl->rssi:-127;
  portEXIT_CRITICAL(&rxMux);
 }else if(len==(int)sizeof(AccessoryPacket)){
  AccessoryPacket a;memcpy(&a,data,sizeof(a));
  if(a.magic!=MAGIC_EXTRA||a.version!=EXTRA_VERSION||a.type!=3||a.gear>2||a.rpmRequest>1000)return;
  portENTER_CRITICAL(&rxMux);
  rxExtra=a;extraAtMs=millis();haveExtra=true;
  portEXIT_CRITICAL(&rxMux);
 }
}

Preferences prefs;
Servo steering,esc,hood;
#if EXHAUST_ACTUATOR_ENABLED
Servo exhaustServo;
#endif
Adafruit_NeoPixel strip(LED_COUNT,STRIP_PIN,NEO_GRB+NEO_KHZ800);
int servoLeft=1300,servoCenter=1500,servoRight=1700;
int stagedLeft=1300,stagedRight=1700;
uint16_t actionAck=0;uint8_t actionError=0;
uint32_t servoSafetyAt=0;
void loadServo(){
 prefs.begin("rc_servo_v4",false);
 servoLeft=prefs.getInt("left",1300);servoCenter=prefs.getInt("center",1500);servoRight=prefs.getInt("right",1700);
 bool good=(servoLeft>=1000&&servoLeft<=2000&&servoCenter>=1000&&servoCenter<=2000&&servoRight>=1000&&servoRight<=2000)&&
   ((servoLeft+15<servoCenter&&servoCenter+15<servoRight)||(servoRight+15<servoCenter&&servoCenter+15<servoLeft));
 if(!good){servoLeft=1300;servoCenter=1500;servoRight=1700;}
 stagedLeft=servoLeft;stagedRight=servoRight;
}
int steerOutput(float x){
 x=constrain(x,-1.0f,1.0f);
 int v=x<0?servoCenter+(int)lroundf(x*(servoCenter-servoLeft)):
               servoCenter+(int)lroundf(x*(servoRight-servoCenter));
 return constrain(v,1000,2000);
}

// ----- Gyro MPU6500/9250: retains V4.1 receiver yaw compensation -----
uint8_t imuAddr=0x68,gyroWho=0;bool gyroOK=false;float gyroBias=0,gyroDps=0;
uint32_t gyroLastUs=0;volatile uint8_t activeGyroFilter=25;
uint32_t gyroReadErrors=0,gyroReadOK=0;
volatile uint32_t gyroIrq=0;void IRAM_ATTR gyroInterrupt(){gyroIrq++;}
bool imuWrite(uint8_t r,uint8_t v){Wire.beginTransmission(imuAddr);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
bool imuRead(uint8_t r,uint8_t &v){Wire.beginTransmission(imuAddr);Wire.write(r);if(Wire.endTransmission(false)!=0)return false;
 if(Wire.requestFrom((int)imuAddr,1)!=1)return false;v=(uint8_t)Wire.read();return true;}
bool readYaw(int16_t &v){Wire.beginTransmission(imuAddr);Wire.write(0x47);if(Wire.endTransmission(false)!=0)return false;
 if(Wire.requestFrom((int)imuAddr,2)!=2)return false;uint8_t hi=Wire.read(),lo=Wire.read();v=(int16_t)((uint16_t(hi)<<8)|lo);return true;}
bool startGyro(bool slowBus=false){
 Wire.begin(IMU_SDA,IMU_SCL);Wire.setClock(slowBus?100000:400000);Wire.setTimeOut(20);
 uint8_t who=0;for(uint8_t addr:{uint8_t(0x68),uint8_t(0x69)}){imuAddr=addr;uint8_t t=0;if(imuRead(0x75,t)&&(t==0x70||t==0x71||t==0x73)){who=t;break;}}
 Serial.printf("[GYRO] I2C addr=0x%02X WHO_AM_I=0x%02X\n",imuAddr,who);
 if(!who)return false;
 gyroWho=who;
 if(!imuWrite(0x6B,0x80))return false;delay(100);
 if(!imuWrite(0x6B,0x01))return false;delay(20);
 if(!imuWrite(0x6C,0)||!imuWrite(0x1A,0x01)||!imuWrite(0x19,0)||!imuWrite(0x1B,0x18)||!imuWrite(0x38,1))return false;
 int64_t sum=0;int count=0;for(int i=0;i<500;i++){int16_t raw=0;if(readYaw(raw)){sum+=raw;count++;}delay(2);}
 if(count<350){Serial.printf("[GYRO] startup sampling FAIL samples=%d/500\n",count);return false;}
 gyroBias=float(sum)/count;gyroLastUs=micros();
 pinMode(IMU_INT,INPUT);attachInterrupt(digitalPinToInterrupt(IMU_INT),gyroInterrupt,RISING);
 Serial.printf("[GYRO] ready who=0x%02X bias=%.2f\n",who,gyroBias);return true;
}
void tickGyro(uint8_t filter){
 if(!gyroOK)return;uint32_t now=micros();
 // 400Hz IMU updates are faster than the 50Hz steering servo and free CPU for WiFi.
 static uint32_t lastAttemptUs=0;
 if((uint32_t)(now-lastAttemptUs)<2500)return;lastAttemptUs=now;
 int16_t raw=0;if(readYaw(raw)){
  float alpha=constrain(0.06f+float(filter)*0.0040f,0.06f,0.46f);
  float dps=(float(raw)-gyroBias)/16.4f;
  gyroDps+=alpha*(dps-gyroDps);gyroLastUs=micros();gyroReadOK++;
 }else gyroReadErrors++;
}

// ----- Motor A/B quadrature counter -----
volatile int32_t encoderTicks=0;volatile uint8_t prevAB=0;
void IRAM_ATTR encoderISR(){uint8_t ab=(digitalRead(RPM_A_PIN)<<1)|digitalRead(RPM_B_PIN);
 uint8_t k=(prevAB<<2)|ab;prevAB=ab;
 if(k==1||k==7||k==14||k==8)encoderTicks++;
 else if(k==2||k==11||k==13||k==4)encoderTicks--;}
float realMotorRPM=0;int32_t lastTicks=0;uint32_t rpmAt=0;
void updateRPM(uint32_t now){if(now-rpmAt<100)return;
 int32_t ticks;noInterrupts();ticks=encoderTicks;interrupts();
 uint32_t dt=now-rpmAt;int32_t delta=ticks-lastTicks;lastTicks=ticks;rpmAt=now;
 float rpm=fabsf(float(delta)*60000.0f/(float(MOTOR_ENCODER_PPR_X4)*dt));
 realMotorRPM+=0.30f*(rpm-realMotorRPM);
}

// ----- Engine state machine, 2-second starter, 2.4-second shutdown -----
enum EngineState:uint8_t{ENGINE_OFF,ENGINE_CRANK,ENGINE_RUN,ENGINE_STOPPING};
volatile EngineState engineState=ENGINE_OFF;
volatile uint32_t engineSince=0;
float virtualRPM=900,engineThrot=0,engineGain=0;
volatile float sharedRPM=900,sharedThrottle=0,sharedEngineGain=0;
volatile float sharedEngineVolume=0.7f,sharedExhaustVolume=0.7f;
volatile bool eventBov=false,eventPop=false,eventHorn=false,eventIndicator=false;
uint8_t lastHornCounter=0;bool knownHorn=false;
// Original TX V6 only transmits short button releases. RX-only neutral double-tap
// replaces the unavailable 2-second hold detection without changing TX firmware.
uint32_t neutralHornFirstAt=0;
static constexpr uint32_t NEUTRAL_DOUBLE_TAP_MS=650;
uint32_t lastPopAt=0,lastHornAt=0,delayedPopAt=0;

void requestStart(uint32_t now){if(engineState==ENGINE_OFF){engineState=ENGINE_CRANK;engineSince=now;
 Serial.println("[ENGINE] START 2000ms crank");}}
void requestStop(uint32_t now){if(engineState==ENGINE_RUN){engineState=ENGINE_STOPPING;engineSince=now;
 Serial.println("[ENGINE] STOP fading 2400ms");}}
void updateEngine(uint32_t now,bool extraFresh,const AccessoryPacket &a){
 float throttle=(extraFresh?float(a.rpmRequest)/1000.0f:0.0f);
 if(engineState==ENGINE_CRANK && now-engineSince>=2000){engineState=ENGINE_RUN;engineSince=now;}
 if(engineState==ENGINE_STOPPING && now-engineSince>=2400){engineState=ENGINE_OFF;}
 float target=900;
 if(engineState==ENGINE_RUN){
  if(extraFresh && a.gear!=1 && realMotorRPM>150){target=constrain(900+realMotorRPM*1.1f,900.0f,6800.0f);}
  else target=900+throttle*5900;
 }
 if(engineState==ENGINE_CRANK)target=200+500*float(now-engineSince)/2000.0f;
 if(engineState==ENGINE_STOPPING)target=900*(1.0f-float(now-engineSince)/2400.0f);
 virtualRPM+=0.05f*(target-virtualRPM);
 engineThrot+=0.12f*(throttle-engineThrot);
 float goal=(engineState==ENGINE_RUN)?1.0f:(engineState==ENGINE_CRANK)?0.20f:
   (engineState==ENGINE_STOPPING?constrain(1.0f-float(now-engineSince)/2400.0f,0.0f,1.0f):0.0f);
 engineGain+=0.10f*(goal-engineGain);
 sharedRPM=virtualRPM;sharedThrottle=engineThrot;sharedEngineGain=engineGain;
 sharedEngineVolume=extraFresh?float(a.engineVol)/100.0f:0.7f;
 sharedExhaustVolume=extraFresh?float(a.exhaustVol)/100.0f:0.7f;
}

void updateHorn(uint32_t now,bool fresh,const AccessoryPacket &a){
 if(!fresh){neutralHornFirstAt=0;return;}
 // All eight hornToggle bits belong to the existing TX press counter.
 // The TX does not transmit a long-hold flag.
 const uint8_t count=a.hornToggle;
 if(!knownHorn){lastHornCounter=count;knownHorn=true;}
 else if(count!=lastHornCounter){
  lastHornCounter=count;
  if(engineState==ENGINE_OFF){
   neutralHornFirstAt=0;
   requestStart(now);
  }else if(engineState==ENGINE_RUN && a.gear==1){
   if(neutralHornFirstAt && (uint32_t)(now-neutralHornFirstAt)<=NEUTRAL_DOUBLE_TAP_MS){
    neutralHornFirstAt=0;
    requestStop(now);
    Serial.println("[ENGINE] RX-only N double-tap shutdown");
   }else neutralHornFirstAt=now;
  }else{
   neutralHornFirstAt=0;
   if(now-lastHornAt>140){eventHorn=true;lastHornAt=now;}
  }
 }
 if(neutralHornFirstAt && (uint32_t)(now-neutralHornFirstAt)>=NEUTRAL_DOUBLE_TAP_MS){
  neutralHornFirstAt=0;
  if(engineState==ENGINE_RUN && a.gear==1){eventHorn=true;lastHornAt=now;}
 }
}
// ----- Audio, GPIO25 front and GPIO26 rear, dual 8-bit DAC -----
// The original BAC Mono WAV loops are converted to 16-bit, 16-kHz PCM in GitHub Actions.
// I2S stereo DAC can later replace this backend without changing engine logic.
uint32_t noiseSeed=0xA56B731DU;
inline float noise01(){noiseSeed^=noiseSeed<<13;noiseSeed^=noiseSeed>>17;noiseSeed^=noiseSeed<<5;
 return ((noiseSeed&65535)/32768.0f)-1.0f;}
inline float sat(float v){return constrain(v,-1.0f,1.0f);}
#if HAS_REAL_ENGINE_SAMPLES
struct PlayLoop{const int16_t *wav;uint32_t len;float phase=0;};
PlayLoop loops[4]={{engine_on_low,ENGINE_ON_LOW_LEN,0},{engine_on_high,ENGINE_ON_HIGH_LEN,0},
 {engine_off_low,ENGINE_OFF_LOW_LEN,0},{engine_off_high,ENGINE_OFF_HIGH_LEN,0}};
float loopSample(PlayLoop &s,float rate){
 if(!s.len)return 0;
 uint32_t i=(uint32_t)s.phase;
 uint32_t next=i+1>=s.len?0:i+1;
 float fract=s.phase-float(i);
 int16_t a=(int16_t)pgm_read_word(s.wav+i),b=(int16_t)pgm_read_word(s.wav+next);
 s.phase+=rate;
 if(s.phase>=s.len)s.phase-=s.len;
 return (a+(b-a)*fract)/32768.0f;
}
#endif
void audioTask(void*){
 float spool=0,bovEnv=0,popEnv=0,hornEnv=0,clickEnv=0,lowLP=0;
 float popPhase=0,hornPhase=0,crankPhase=0,enginePhase=0;
 float lowVol=1.0f,highVol=0.0f,onVol=0.0f,offVol=1.0f,pitch=1.0f;
 int64_t next=esp_timer_get_time();
 uint32_t sampleIndex=0;
 for(;;){
  float rpm=constrain(sharedRPM,100.0f,7200.0f);
  float throttle=constrain(sharedThrottle,0.0f,1.0f);
  float engGain=constrain(sharedEngineGain*sharedEngineVolume,0.0f,1.0f);
  float rearGain=constrain(sharedExhaustVolume,0.0f,1.0f);
  if(eventBov){eventBov=false;bovEnv=1.0f;}
  if(eventPop){eventPop=false;popEnv=1.0f;popPhase=0;}
  if(eventHorn){eventHorn=false;hornEnv=1.0f;hornPhase=0;}
  if(eventIndicator){eventIndicator=false;clickEnv=1.0f;}
  float expectedSpool=(engineState==ENGINE_RUN)?throttle*constrain((rpm-1100)/4400.0f,0.0f,1.0f):0.0f;
  spool+=0.00035f*(expectedSpool-spool);
  float growl=0;
#if HAS_REAL_ENGINE_SAMPLES
  // Expensive trig/exponential envelope calculations only at 250Hz,
  // not 16000 times/sec: greatly reduces core0 contention with WiFi.
  if((sampleIndex&63U)==0){
   float high=constrain((rpm-3000)/3500.0f,0.0f,1.0f);
   lowVol=cosf(high*1.5707963f);highVol=sinf(high*1.5707963f);
   onVol=sinf(throttle*1.5707963f);offVol=cosf(throttle*1.5707963f);
   pitch=powf(2.0f,(rpm-1000.0f)*0.2f/1200.0f);
  }
  growl=(loopSample(loops[0],pitch)*lowVol*onVol+
         loopSample(loops[1],pitch)*highVol*onVol+
         loopSample(loops[2],pitch)*lowVol*offVol+
         loopSample(loops[3],pitch)*highVol*offVol)*0.70f;
#else
  enginePhase+=6.2831853f*(rpm/60.0f*3.0f)/AUDIO_RATE;
  if(enginePhase>6.2831853f)enginePhase-=6.2831853f;
  growl=(sinf(enginePhase)+0.48f*sinf(2.0f*enginePhase)+0.18f*sinf(3.0f*enginePhase))*0.20f;
#endif
  float crank=0;
  if(engineState==ENGINE_CRANK){crankPhase+=6.2831853f*(24+rpm*0.018f)/AUDIO_RATE;
   crank=(0.48f*sinf(crankPhase)+0.25f*noise01())*0.33f;}
  // Front: real loop + bass enhancement + spool; around 2x stronger than first simulator.
  lowLP+=0.020f*(growl-lowLP);
  float turbo=0.0f;
  float n=noise01();
  float whistlePhase=6.2831853f*(600+1800*spool)*float(sampleIndex%16000)/AUDIO_RATE;
  turbo=(sinf(whistlePhase)*0.13f+n*0.055f)*spool*2.0f;
  float front=sat((growl*0.8f+lowLP*0.55f)*engGain+turbo*engGain+crank);
  // Rear: BOV shshsh decay ~1.5s, double pop, horn and signal clicking.
  float rearNoise=noise01();
  float bov=rearNoise*0.49f*bovEnv;
  bovEnv*=0.999875f; if(bovEnv<0.0001f)bovEnv=0;
  float pop=0;
  if(popEnv>0.0001f){popPhase+=1.0f/AUDIO_RATE;
   float spike=expf(-popPhase*33.0f)+0.8f*expf(-fabsf(popPhase-0.18f)*38.0f);
   pop=(rearNoise*0.50f+sinf(popPhase*260.0f)*0.34f)*spike;
   if(popPhase>0.40f)popEnv=0;
  }
  hornPhase+=2.0f*3.14159265f*440.0f/AUDIO_RATE;
  float horn=(sinf(hornPhase)+0.48f*sinf(hornPhase*1.5f))*0.20f*hornEnv;
  hornEnv*=0.99984f; if(hornEnv<0.0001f)hornEnv=0;
  float click=rearNoise*0.12f*clickEnv;
  clickEnv*=0.986f;if(clickEnv<0.0001f)clickEnv=0;
  float rear=sat((bov+pop+horn+click)*rearGain);
  dacWrite(FRONT_DAC_PIN,constrain((int)lroundf(128+front*115),0,255));
  dacWrite(REAR_DAC_PIN,constrain((int)lroundf(128+rear*115),0,255));
  ++sampleIndex;
  next+=1000000LL/AUDIO_RATE;
  int64_t now=esp_timer_get_time();
  if(next<now-10000)next=now; // resynchronise after Wi-Fi/scheduler delays
  while((now=esp_timer_get_time())<next){if(next-now>700)delayMicroseconds(50);}
  // Allow idle/watchdog and lower-priority WiFi maintenance tasks to run.
  // DMA/I2S is the preferred hardware upgrade for truly glitch-free sound.
  if((sampleIndex&1023U)==0)vTaskDelay(1);
 }
}

// ----- Lighting index layout: total 17 GRB pixels -----
// 0 RF turn | 1-2 R headlights | 3-4 L headlights | 5 LF turn | 6 LR turn
// 7-8 L brakes | 9-10 R brakes | 11 RR turn | 12 exhaust | 13-16 underglow.
uint32_t rgb(uint8_t r,uint8_t g,uint8_t b){return strip.Color(r,g,b);}
void setPair(uint8_t i,uint8_t j,uint32_t c){strip.setPixelColor(i,c);strip.setPixelColor(j,c);}
uint32_t lastLights=0;bool lastBlink=false;
void lights(uint32_t now,bool fresh,const AccessoryPacket &a){
 if(now-lastLights<35)return;lastLights=now;
 bool blink=(now/400)%2==0;
 bool haz=fresh&&a.hazard;
 bool left=fresh&&a.leftSignal,right=fresh&&a.rightSignal;
 bool turnL=(haz||left)&&blink,turnR=(haz||right)&&blink;
 bool headlights=fresh&&a.headlights;
 uint8_t braking=fresh?a.brakeLevel:0;
 uint8_t br=constrain(fresh?(int)a.lightBrightness:70,0,100);
 auto level=[br](int n)->uint8_t{return (uint8_t)constrain(n*br/100,0,255);};
 strip.clear();
 uint32_t amber=rgb(level(255),level(85),0);
 strip.setPixelColor(0,turnR?amber:0);
 setPair(1,2,headlights?rgb(level(220),level(220),level(255)):0);
 setPair(3,4,headlights?rgb(level(220),level(220),level(255)):0);
 strip.setPixelColor(5,turnL?amber:0);
 strip.setPixelColor(6,turnL?amber:0);
 uint32_t stop=rgb(level(braking*255/100),0,0);
 setPair(7,8,stop);setPair(9,10,stop);
 strip.setPixelColor(11,turnR?amber:0);
 uint32_t exhaustLight=(engineState==ENGINE_RUN)?rgb(level(80),level(18),0):0;
 strip.setPixelColor(12,exhaustLight);
 uint32_t floor=(headlights||engineState==ENGINE_RUN)?rgb(0,level(5),level(35)):0;
 for(int i=13;i<17;i++)strip.setPixelColor(i,floor);
 strip.show();
 if(blink && !lastBlink && (haz||left||right))eventIndicator=true;
 lastBlink=blink;
}

// ----- GPIO13 actuator: one-wire servo-style motor controller only -----
#if EXHAUST_ACTUATOR_ENABLED
uint32_t actuatorStopAt=0;
void pulseActuator(uint32_t now){exhaustServo.writeMicroseconds(1350);actuatorStopAt=now+100;}
void tickActuator(uint32_t now){if(actuatorStopAt && (int32_t)(now-actuatorStopAt)>=0){exhaustServo.writeMicroseconds(1650);actuatorStopAt=0;}}
#else
void pulseActuator(uint32_t){}
void tickActuator(uint32_t){}
#endif

uint32_t lastControl=0,lastTel=0,lastStats=0;
uint16_t batMV=0;
bool armed=false;uint32_t neutralFrom=0;
int steeringUs=1500,escUs=1500;
void processAction(const ControlPacket &c){
 if(c.actionCode==0||!c.actionId||c.actionId==actionAck||c.mode!=1||c.actionCode>3)return;
 int v=c.previewValue;actionError=0;
 if(v<1000||v>2000){actionError=1;actionAck=c.actionId;return;}
 if(c.actionCode==1)stagedLeft=v;
 else if(c.actionCode==2)stagedRight=v;
 else if(c.actionCode==3){
  if(!((stagedLeft+15<v&&v+15<stagedRight)||(stagedRight+15<v&&v+15<stagedLeft))){actionError=2;actionAck=c.actionId;return;}
  servoLeft=stagedLeft;servoRight=stagedRight;servoCenter=v;
  bool ok=prefs.putInt("left",servoLeft)==4&&prefs.putInt("center",servoCenter)==4&&prefs.putInt("right",servoRight)==4;
  if(!ok)actionError=3;
 }
 actionAck=c.actionId;
}
int throttlePulse(const ControlPacket &c){
 float limit=float(c.speedPot)/1000.0f*float(c.throttle>=0?c.maxForward:c.maxReverse)/1000.0f;
 return constrain((int)lroundf(1500.0f+c.throttle*0.5f*limit),1000,2000);
}
void control(uint32_t nowMs){
 if(nowMs-lastControl<4)return;lastControl=nowMs;
 ControlPacket c;AccessoryPacket a;uint32_t at,atExtra;bool have,haveAcc;
 portENTER_CRITICAL(&rxMux);
 c=rxControl;a=rxExtra;at=rxAtUs;atExtra=extraAtMs;have=haveControl;haveAcc=haveExtra;
 portEXIT_CRITICAL(&rxMux);
 uint32_t nowUs=micros();bool live=have && (uint32_t)(nowUs-at)<FAILSAFE_US;
 bool fresh=haveAcc && (nowMs-atExtra)<EXTRA_TIMEOUT_MS && live;
 if(!live){armed=false;neutralFrom=0;escUs=ESC_NEUTRAL;steeringUs=servoCenter;}
 else{
  processAction(c);
  bool thrCenter=abs((int)c.throttle)<=15;
  if(!armed){if(thrCenter){if(!neutralFrom)neutralFrom=nowMs;if(nowMs-neutralFrom>=800)armed=true;}else neutralFrom=0;}
  if(c.mode==1){steeringUs=constrain((int)c.previewValue,1000,2000);escUs=ESC_NEUTRAL;}
  else if(c.mode==3){steeringUs=servoCenter;escUs=ESC_NEUTRAL;}
  else{
   float correction=0;
   if(c.mode==0&&gyroOK&&(uint32_t)(micros()-gyroLastUs)<150000 && (!fresh||a.gyroEnabled)){
    float gain=float(c.gyroSetting)/1000.0f*float(c.gyroPot)/1000.0f;
    if(fresh)gain*=float(a.gyroGain)/100.0f*2.0f;
    float rev=fresh&&a.gyroReverse?-1.0f:1.0f;
    float desired=constrain(rev*gyroDps*0.45f*gain/20.0f,-1.0f,1.0f);
    static float smooth=0;
    float alpha=constrain(1.0f-(fresh?float(a.gyroDamping):20.0f)/120.0f,0.12f,1.0f);
    smooth+=alpha*(desired-smooth);
    correction=smooth;
   }
   float manual=float(c.steer)/1000.0f;if(c.flags&1){manual=-manual;correction=-correction;}
   steeringUs=steerOutput(manual+correction);
   escUs=(armed&&c.mode==0)?throttlePulse(c):ESC_NEUTRAL;
  }
 }
 steering.writeMicroseconds(steeringUs);esc.writeMicroseconds(escUs);
 activeGyroFilter=fresh?a.gyroFilter:25;
 updateHorn(nowMs,fresh,a);
 if(fresh){
  int hoodAngle=a.hoodOpen?90:15;
  static int previousHood=-1;if(hoodAngle!=previousHood){hood.write(hoodAngle);previousHood=hoodAngle;}
  static int lastThrot=0;int current=a.rpmRequest;
  if(lastThrot>=300&&lastThrot-current>=200&&engineState==ENGINE_RUN&&nowMs-lastPopAt>600){
   eventBov=true;delayedPopAt=nowMs+320;lastPopAt=nowMs;
  }
  lastThrot=current;
 }else{static int prior=-1;if(prior!=15){hood.write(15);prior=15;}}
 updateEngine(nowMs,fresh,a);
 lights(nowMs,fresh,a);
 if(delayedPopAt && (int32_t)(nowMs-delayedPopAt)>=0){delayedPopAt=0;eventPop=true;pulseActuator(nowMs);}
 tickActuator(nowMs);
}
void telemetry(uint32_t nowMs){
 if(nowMs-lastTel<100)return;lastTel=nowMs;
 if(sendingTelemetry){if(nowMs-sendStarted>250)sendingTelemetry=false;else return;}
 // Register one unicast peer from the control packets. Unlike broadcast,
 // unicast provides MAC-layer acknowledgements and retry handling.
 if(txMacKnown && (!txPeerRegistered || memcmp(txMac,activeTxMac,6)!=0)
    && nowMs-lastTXPeerAttempt>1000){
  lastTXPeerAttempt=nowMs;
  uint8_t target[6];portENTER_CRITICAL(&rxMux);memcpy(target,activeTxMac,6);portEXIT_CRITICAL(&rxMux);
  if(txPeerRegistered)esp_now_del_peer(txMac);
  esp_now_peer_info_t peer={};memcpy(peer.peer_addr,target,6);
  peer.channel=RADIO_CHANNEL;peer.ifidx=WIFI_IF_STA;peer.encrypt=false;
  esp_err_t result=esp_now_add_peer(&peer);
  txPeerRegistered=(result==ESP_OK||result==ESP_ERR_ESPNOW_EXIST);
  if(txPeerRegistered){memcpy(txMac,target,6);Serial.printf("[RADIO] TX unicast peer %02X:%02X:%02X:%02X:%02X:%02X\n",target[0],target[1],target[2],target[3],target[4],target[5]);}
 }
 uint32_t at,total;int8_t rssi;bool seen;
 portENTER_CRITICAL(&rxMux);at=rxAtUs;total=packetCount;rssi=lastRSSI;seen=haveControl;portEXIT_CRITICAL(&rxMux);
 TelemetryV5 t={};t.magic=MAGIC_LEGACY;t.version=LEGACY_VERSION;t.type=2;
 t.linked=seen && uint32_t(micros()-at)<FAILSAFE_US;
 t.gyroOK=gyroOK && uint32_t(micros()-gyroLastUs)<150000;
 t.rxPps=packetsPerSec;t.batteryMv=batMV;t.rxRssi=rssi;t.armed=armed;
 t.actionAck=actionAck;t.actionError=actionError;
 t.echoTimestampUs=t.linked?rxControl.echoTimestampUs:0;
 t.servoLeftUs=servoLeft;t.servoCenterUs=servoCenter;t.servoRightUs=servoRight;
 sendingTelemetry=true;sendStarted=nowMs;
 esp_err_t e=esp_now_send(txPeerRegistered?txMac:BROADCAST_MAC,(uint8_t*)&t,sizeof(t));
 if(e!=ESP_OK){sendingTelemetry=false;telemetrySendFailures++;}
}
void startRadio(){
 WiFi.mode(WIFI_STA);WiFi.setSleep(false);esp_wifi_set_ps(WIFI_PS_NONE);
 esp_wifi_set_channel(RADIO_CHANNEL,WIFI_SECOND_CHAN_NONE);
 if(esp_now_init()!=ESP_OK){Serial.println("[RADIO] ESP-NOW init failed");return;}
 esp_now_register_recv_cb(onReceive);esp_now_register_send_cb(onSent);
 esp_now_peer_info_t peer={};memcpy(peer.peer_addr,BROADCAST_MAC,6);
 peer.ifidx=WIFI_IF_STA;peer.channel=RADIO_CHANNEL;peer.encrypt=false;
 esp_err_t res=esp_now_add_peer(&peer);
 Serial.printf("[RADIO] ESP-NOW broadcast peer=%d CH=%u\n",(int)res,RADIO_CHANNEL);
}
void setup(){
 Serial.begin(115200);delay(200);
 Serial.println("RC DRIFT RX V6 / AUDIO ENGINE / PROTOCOL 5 + 6");
 pinMode(EXHAUST_PIN,OUTPUT);digitalWrite(EXHAUST_PIN,LOW);
 pinMode(RPM_A_PIN,INPUT_PULLUP);pinMode(RPM_B_PIN,INPUT_PULLUP);
 prevAB=(digitalRead(RPM_A_PIN)<<1)|digitalRead(RPM_B_PIN);
 attachInterrupt(digitalPinToInterrupt(RPM_A_PIN),encoderISR,CHANGE);
 attachInterrupt(digitalPinToInterrupt(RPM_B_PIN),encoderISR,CHANGE);
 loadServo();
 steering.setPeriodHertz(50);esc.setPeriodHertz(50);hood.setPeriodHertz(50);
 steering.attach(SERVO_PIN,1000,2000);esc.attach(ESC_PIN,1000,2000);hood.attach(HOOD_PIN,500,2500);
 steering.writeMicroseconds(servoCenter);esc.writeMicroseconds(ESC_NEUTRAL);hood.write(15);
#if EXHAUST_ACTUATOR_ENABLED
 exhaustServo.setPeriodHertz(50);exhaustServo.attach(EXHAUST_PIN,1000,2000);exhaustServo.writeMicroseconds(1500);
#endif
 strip.begin();strip.clear();strip.show();
 analogReadResolution(12);analogSetPinAttenuation(BATTERY_PIN,ADC_11db);
 gyroOK=startGyro();
 if(!gyroOK){
  Serial.println("[GYRO] first init failed: retry with 100kHz I2C after power settles");
  delay(100);gyroOK=startGyro(true);
 }
 Serial.printf("[GYRO] init=%d addr=0x%02X WHO=0x%02X\n",gyroOK,imuAddr,gyroWho);
 dacWrite(FRONT_DAC_PIN,128);dacWrite(REAR_DAC_PIN,128);
 // Bring up ESP-NOW before starting CPU-intensive audio generation.
 rpmAt=millis();startRadio();
 xTaskCreatePinnedToCore(audioTask,"rx_audio",6144,nullptr,2,nullptr,0);
 Serial.printf("[AUDIO] bank=%s rate=%d GPIO25/26, LEDs=%d, exhaustActuator=%d\n",
   HAS_REAL_ENGINE_SAMPLES?"BAC original":"synth fallback",AUDIO_RATE,LED_COUNT,EXHAUST_ACTUATOR_ENABLED);
}
void loop(){
 uint32_t now=millis();
 tickGyro(activeGyroFilter);control(now);updateRPM(now);telemetry(now);
 static uint32_t batAt=0;if(now-batAt>=500){batAt=now;
  uint32_t sum=0;for(int i=0;i<4;i++)sum+=analogReadMilliVolts(BATTERY_PIN);
  batMV=(uint16_t)constrain((int)lroundf((sum/4.0f)*3.0f),0,20000);
 }
 if(now-lastStats>=1000){lastStats=now;
  uint32_t total;portENTER_CRITICAL(&rxMux);total=packetCount;portEXIT_CRITICAL(&rxMux);
  packetsPerSec=constrain((int)(total-lastPackets),0,65535);lastPackets=total;
  Serial.printf("[RX] LINK=%d AGE=%lums PPS=%u RSSI=%ddBm GYRO=%d G_AGE=%lums I2C_ERR=%lu G_READ=%lu TEL_CB=%lu TEL_FAIL=%lu TEL_SEND_ERR=%lu RPM=%.0f ESC=%d HEAP=%u RESET=%d\n",
    haveControl&&(uint32_t)(micros()-rxAtUs)<FAILSAFE_US,
    (unsigned long)((uint32_t)(micros()-rxAtUs)/1000U),
    packetsPerSec,(int)lastRSSI,(int)gyroOK,
    (unsigned long)((uint32_t)(micros()-gyroLastUs)/1000U),
    (unsigned long)gyroReadErrors,(unsigned long)gyroReadOK,
    (unsigned long)telemetryCallbacks,(unsigned long)telemetryDeliveryFailures,
    (unsigned long)telemetrySendFailures,
    realMotorRPM,escUs,(unsigned)ESP.getFreeHeap(),(int)esp_reset_reason());
 }
 delay(1);
}