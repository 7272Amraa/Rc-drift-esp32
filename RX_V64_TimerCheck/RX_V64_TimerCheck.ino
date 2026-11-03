#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <driver/dac_oneshot.h>
#include <soc/soc.h>
#include <soc/rtc_io_reg.h>
#include <esp32-hal-timer.h>

static constexpr int AUDIO_RATE=16000;
static constexpr uint32_t RING=1024,BLK=128;
static DRAM_ATTR uint16_t audioRing[RING];
static volatile uint32_t rd=0,wr=0,underflows=0,ticks=0;
static hw_timer_t *timerHandle=nullptr;
static dac_oneshot_handle_t dac1=nullptr,dac2=nullptr;
static void IRAM_ATTR tick(){
  uint32_t r=rd;
  uint32_t w=__atomic_load_n(&wr,__ATOMIC_ACQUIRE);
  uint16_t sample=0x8080;
  if(r!=w){sample=audioRing[r&(RING-1)];rd=r+1;}
  else underflows++;
  REG_SET_FIELD(RTC_IO_PAD_DAC1_REG,RTC_IO_PDAC1_DAC,(uint8_t)sample);
  REG_SET_FIELD(RTC_IO_PAD_DAC2_REG,RTC_IO_PDAC2_DAC,(uint8_t)(sample>>8));
  ticks++;
}
static bool ready(){
  dac_oneshot_config_t c0={};c0.chan_id=DAC_CHAN_0;
  dac_oneshot_config_t c1={};c1.chan_id=DAC_CHAN_1;
  if(dac_oneshot_new_channel(&c0,&dac1)!=ESP_OK) return false;
  if(dac_oneshot_new_channel(&c1,&dac2)!=ESP_OK) return false;
  dac_oneshot_output_voltage(dac1,128);
  dac_oneshot_output_voltage(dac2,128);
  timerHandle=timerBegin(AUDIO_RATE);
  if(!timerHandle)return false;
  timerAttachInterrupt(timerHandle,&tick);
  for(int i=0;i<512;i++)audioRing[i]=(uint16_t)(128|(128<<8));
  __atomic_store_n(&wr,(uint32_t)512,__ATOMIC_RELEASE);
  timerAlarm(timerHandle,1,true,0);
  return true;
}
void setup(){
  Serial.begin(115200);
  Serial.printf("DAC audio timer: %s\n",ready()?"OK":"FAIL");
}
void loop(){
  uint32_t count=__atomic_load_n(&wr,__ATOMIC_ACQUIRE)-__atomic_load_n(&rd,__ATOMIC_ACQUIRE);
  if(count<512){
    uint32_t w=wr;
    for(int i=0;i<BLK;i++){
      float a=(w+i)*2.0f*3.141592653589793f*500.0f/AUDIO_RATE;
      uint8_t s=(uint8_t)constrain((int)lroundf(128+24*sinf(a)),0,255);
      audioRing[(w+i)&(RING-1)]=(uint16_t)s|((uint16_t)s<<8);
    }
    __atomic_store_n(&wr,w+BLK,__ATOMIC_RELEASE);
  }
  delay(1);
}