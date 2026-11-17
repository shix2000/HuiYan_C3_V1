#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <atomic>
#include "PairButton.h"
#include "Gesture.h"
#include "TiltMouse.h"

#define SDA_PIN 4
#define SCL_PIN 5
#define PINCH_PIN 3

// CSV remains SI: acceleration m/s^2, gyro rad/s.
Adafruit_MPU6050 mpu;
uint8_t imuAddress=0x68;
bool gesturesEnabled=true;
bool gestureDebug=true;
constexpr uint32_t SAMPLE_MS=20, DEBOUNCE_MS=25, DOUBLE_MS=300;
constexpr uint32_t SHORT_MS=500, LONG_MS=3000, RELEASE_MS=30;
NimBLEServer* server;
NimBLECharacteristic *keyboardReport, *mouseReport;
std::atomic<uint16_t> activeHandle{BLE_HS_CONN_HANDLE_NONE};
std::atomic<bool> keySubscribed{false}, mouseSubscribed{false};
std::atomic<uint32_t> linkEpoch{0}, pairUntil{0};
bool imuReady=false, recording=false, mouseMode=false, ready=false;
bool recordWaitRelease=true, recordActive=false;
bool rawLast=HIGH, stableState=HIGH, waitRelease=true;
uint32_t lastChange=0, lastSample=0, downAt=0, upAt=0, seenEpoch=0;
enum class Button { Idle, Down1, Wait2, Down2 };
Button button=Button::Idle;
bool releasePending=false;
uint32_t pressedAt=0;
uint32_t mouseBlockedAt=0;

// One HOGP service, keyboard ID 1 (8 bytes), relative mouse ID 2 (3 bytes).
static uint8_t reportMap[]={
  0x05,0x01,0x09,0x06,0xA1,0x01,0x85,0x01,
  0x05,0x07,0x19,0xE0,0x29,0xE7,0x15,0x00,0x25,0x01,
  0x75,0x01,0x95,0x08,0x81,0x02,0x75,0x08,0x95,0x01,0x81,0x01,
  0x05,0x08,0x19,0x01,0x29,0x05,0x75,0x01,0x95,0x05,0x91,0x02,
  0x75,0x03,0x95,0x01,0x91,0x01,
  0x05,0x07,0x19,0x00,0x29,0x65,0x15,0x00,0x25,0x65,
  0x75,0x08,0x95,0x06,0x81,0x00,0xC0,
  0x05,0x01,0x09,0x02,0xA1,0x01,0x85,0x02,0x09,0x01,0xA1,0x00,
  0x05,0x09,0x19,0x01,0x29,0x03,0x15,0x00,0x25,0x01,
  0x95,0x03,0x75,0x01,0x81,0x02,0x95,0x01,0x75,0x05,0x81,0x01,
  0x05,0x01,0x09,0x30,0x09,0x31,0x15,0x81,0x25,0x7F,
  0x75,0x08,0x95,0x02,0x81,0x06,0xC0,0xC0
};

bool pairWindow() {
  uint32_t until=pairUntil.load();
  return until && int32_t(until-millis())>0;
}
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& c) override {
    if(s->getConnectedCount()>1){s->disconnect(c.getConnHandle());return;}
    activeHandle=c.getConnHandle();
    keySubscribed=false; mouseSubscribed=false; ++linkEpoch;
    if (!pairWindow() && !NimBLEDevice::isBonded(c.getIdAddress())) {
      s->disconnect(c.getConnHandle()); return;
    }
    NimBLEDevice::startSecurity(c.getConnHandle());
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& c, int) override {
    if(c.getConnHandle()!=activeHandle.load()) return;
    activeHandle=BLE_HS_CONN_HANDLE_NONE;
    keySubscribed=false; mouseSubscribed=false; ++linkEpoch;
  }
  void onAuthenticationComplete(NimBLEConnInfo& c) override {
    if (!c.isEncrypted()) server->disconnect(c.getConnHandle());
  }
} serverCallbacks;
class ReportCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& info, uint16_t value) override {
    if(info.getConnHandle()!=activeHandle.load()) return;
    if(c==keyboardReport) keySubscribed=(value & 1)!=0;
    if(c==mouseReport) mouseSubscribed=(value & 1)!=0;
    ++linkEpoch;
  }
} reportCallbacks;

bool releaseAll() {
  uint8_t k[8]={}, m[3]={};
  bool a=keyboardReport->notify(k,sizeof(k));
  bool b=mouseReport->notify(m,sizeof(m));
  return a && b;
}
void clearInput() {
  button=Button::Idle; waitRelease=true; releasePending=false;
  Gesture::reset();
  TiltMouse::reset();mouseBlockedAt=millis();
}
bool linkUsable() {
  return server->getConnectedCount()==1 && keySubscribed.load() &&
    mouseSubscribed.load() && server->getPeerInfo(uint8_t(0)).isEncrypted();
}
void click(bool doubleClick, uint32_t now) {
  if(!ready || recording || waitRelease || releasePending) return;
  bool sent;
  if(mouseMode) {
    uint8_t m[3]={uint8_t(doubleClick?2:1),0,0};
    sent=mouseReport->notify(m,sizeof(m));
  } else {
    // GPIO3: single = Page Down, double = Page Up (user-confirmed mapping).
    uint8_t k[8]={0,0,uint8_t(doubleClick?0x4B:0x4E),0,0,0,0,0};
    sent=keyboardReport->notify(k,sizeof(k));
  }
  // Even a failed enqueue requires a zero report before accepting more input.
  releasePending=true; pressedAt=now;
  if(!sent) Serial.println("# HID_PRESS_FAILED");
}
void buttonTick(uint32_t now, bool down, bool up) {
  if(waitRelease || !ready || recording) {
    button=Button::Idle;
    if(stableState==HIGH && rawLast==HIGH && now-lastChange>=DEBOUNCE_MS)
      waitRelease=false;
    return;
  }
  // Long wins over up at exactly 3000ms, including a held second press.
  if((button==Button::Down1 || button==Button::Down2) && now-downAt>=LONG_MS) {
    releaseAll(); releasePending=true; pressedAt=now;
    TiltMouse::reset();mouseBlockedAt=now;
    mouseMode=!mouseMode; button=Button::Idle; waitRelease=true;
    Serial.println(mouseMode?"# MODE MOUSE":"# MODE PRESENT"); return;
  }
  switch(button) {
    case Button::Idle:
      if(down){downAt=now; button=Button::Down1;} break;
    case Button::Down1:
      if(up){if(now-downAt<=SHORT_MS){upAt=now; button=Button::Wait2;}
             else button=Button::Idle;} break;
    case Button::Wait2:
      if(down && now-upAt<=DOUBLE_MS){downAt=now; button=Button::Down2;}
      else if(now-upAt>DOUBLE_MS){
        click(false,now); button=Button::Idle;
        if(down){downAt=now; button=Button::Down1;}
      } break;
    case Button::Down2:
      if(up){if(now-downAt<=SHORT_MS) click(true,now); button=Button::Idle;} break;
  }
}
// Both the physical key and serial command use the same pairing policy.
void requestPairing(uint32_t now) {
  if(recording || server->getConnectedCount()!=0){
    Serial.println("# PAIR IGNORED: stop recording and disconnect host first");
    return;
  }
  clearInput();
  pairUntil=now+60000;
  NimBLEDevice::getAdvertising()->start();
  Serial.println("# PAIR WINDOW 60s (Just Works, no MITM protection)");
}
void setup() {
  Serial.begin(115200);
  pinMode(PINCH_PIN,INPUT_PULLUP);
  PairButton::begin();
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(100000);
  Wire.setTimeOut(20);
  imuReady=mpu.begin(imuAddress,&Wire);
  if(!imuReady){imuAddress=0x69;imuReady=mpu.begin(imuAddress,&Wire);}
  if(imuReady){
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  Serial.println("# BLE INIT BEGIN (NimBLE 2.5.2)");
  Serial.flush();
  if (!NimBLEDevice::init("HuiYan V0.4")) {
    Serial.println("# BLE INIT FAILED: HID disabled; restart required");
    // Do not dereference server/report pointers after initialization failure.
    while (true) delay(1000);
  }
  Serial.println("# BLE INIT OK");
  NimBLEDevice::setSecurityAuth(true,false,true); // Bonding, no display/keyboard (Just Works).
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  server=NimBLEDevice::createServer(); server->setCallbacks(&serverCallbacks,false);
  server->advertiseOnDisconnect(true);
  auto hid=new NimBLEHIDDevice(server);
  keyboardReport=hid->getInputReport(1); mouseReport=hid->getInputReport(2);
  hid->getOutputReport(1); // Host keyboard LED report (unused).
  keyboardReport->setCallbacks(&reportCallbacks); mouseReport->setCallbacks(&reportCallbacks);
  hid->setManufacturer("HuiYan Prototype"); hid->setHidInfo(0,1);
  hid->setReportMap(reportMap,sizeof(reportMap)); // Services start with server->start().
  auto adv=NimBLEDevice::getAdvertising();
  adv->setName("HuiYan V0.4"); adv->setAppearance(0x03C0);
  adv->addServiceUUID(hid->getHidService()->getUUID());
  adv->enableScanResponse(true); server->start();
  if(NimBLEDevice::getNumBonds()) adv->start();
  Serial.println("# HuiYan V0.4: p=pair 60s, r=record toggle, h/j/k/l=mouse test, u/d=page up/down test, c=clear bonds");
  Serial.println("# GESTURE: a=auto restore toggle (default ON), n=recalibrate neutral, g=enable toggle, v=diagnostics; wait for READY before rotating");
  Serial.println("# PRESENT GPIO3 single=NEXT double=PREVIOUS; hold 3s toggles MOUSE");
  Serial.println(imuReady?"# IMU READY":"# IMU FAILED: button HID remains available");
}
void loop() {
  uint32_t now=millis();
  if(PairButton::poll(now)) requestPairing(now);
  bool raw=digitalRead(PINCH_PIN), down=false, up=false;
  if(raw!=rawLast) lastChange=now;
  rawLast=raw;
  if(now-lastChange>=DEBOUNCE_MS && raw!=stableState){
    stableState=raw; down=raw==LOW; up=!down;
  }
  uint32_t epoch=linkEpoch.load();
  bool usable=linkUsable();
  if(epoch!=seenEpoch || usable!=ready){
    seenEpoch=epoch; ready=usable; clearInput();
    if(ready){releasePending=true; pressedAt=now-RELEASE_MS;}
    Serial.println(ready?"# HID READY: release button before use":"# HID NOT READY: pending input discarded");
  }
  if(ready && releasePending && now-pressedAt>=RELEASE_MS){
    if(releaseAll()) releasePending=false; // Retry release, never replay a press.
  }
  while(Serial.available()){
    char c=Serial.read();
    if(c=='a'){
      if(!recording){
        Gesture::setAutoRestore(!Gesture::autoRestore());
        Serial.println(Gesture::autoRestore()?"# GST AUTO_RESTORE ON":"# GST AUTO_RESTORE OFF");
      }
    } else if(c=='n'){
      if(recording){Serial.println("# GST CALIBRATION IGNORED: stop recording first");}
      else {
        clearInput();
        if(ready){releaseAll();releasePending=true;pressedAt=now;}
        Serial.println("# RECALIBRATE: release buttons; hold neutral still (mouse 1s; presentation 1.3s)");
      }
    } else if(c=='v'){
      gestureDebug=!gestureDebug;
      Serial.println(gestureDebug?"# GST DEBUG ON":"# GST DEBUG OFF");
    } else if(c=='g'){
      gesturesEnabled=!gesturesEnabled; Gesture::reset();
      Serial.println(gesturesEnabled?"# GESTURE ON: hold neutral 1.3s":"# GESTURE OFF");
    } else if(c=='r'){
      if(recordActive) Serial.println("# PINCH_END");
      recording=!recording; recordWaitRelease=true; recordActive=false; clearInput();
      if(ready){releaseAll();releasePending=true;pressedAt=now;}
      Serial.println(recording?"# RECORD ON: release, then hold to collect":"# RECORD OFF");
      if(recording) Serial.println("time_ms,ax,ay,az,gx,gy,gz");
    } else if(c=='p' && !recording){
      requestPairing(now);
    } else if(c=='c' && !ready && server->getConnectedCount()==0 && !recording){
      NimBLEDevice::getAdvertising()->stop(); pairUntil=0;
      NimBLEDevice::deleteAllBonds(); Serial.println("# BONDS CLEARED; remove host pairing too, then p");
    } else if((c=='u' || c=='d') && ready && !mouseMode && !recording &&
              !waitRelease && button==Button::Idle && stableState==HIGH && !releasePending){
      uint8_t k[8]={0,0,uint8_t(c=='u'?0x4B:0x4E),0,0,0,0,0};
      keyboardReport->notify(k,sizeof(k)); releasePending=true; pressedAt=now;
    } else if(ready && mouseMode && !recording && !waitRelease &&
              button==Button::Idle && stableState==HIGH && !releasePending){
      int8_t x=c=='h'?-10:c=='l'?10:0, y=c=='k'?-10:c=='j'?10:0;
      if(x || y){uint8_t m[3]={0,uint8_t(x),uint8_t(y)};mouseReport->notify(m,sizeof(m));}
    }
  }
  if(pairUntil.load() && !pairWindow()){
    pairUntil=0;
    if(!NimBLEDevice::getNumBonds()) NimBLEDevice::getAdvertising()->stop();
    Serial.println("# PAIR WINDOW CLOSED");
  }
  if(recording){
    if(recordWaitRelease){if(stableState==HIGH) recordWaitRelease=false;}
    else {
      if(down){recordActive=true;Serial.println("# PINCH_START");lastSample=now-SAMPLE_MS;}
      if(up && recordActive){recordActive=false;Serial.println("# PINCH_END");}
    }
  } else buttonTick(now,down,up);
  // Keep sampling outside capture mode; use checked burst reads rather than getEvent's always-true result.
  bool allow=ready && !recording && !mouseMode && !waitRelease &&
    button==Button::Idle && rawLast==HIGH && stableState==HIGH &&
    digitalRead(PairButton::Pin)==HIGH && !pairWindow() && gesturesEnabled;
  Gesture::setDebug(gestureDebug && !recording);
  static uint32_t lastGateLog=0, lastReadErrorLog=0;
  if(gestureDebug && !recording && now-lastGateLog>=1000){
    lastGateLog=now;
    Serial.printf("# GST GATE allow=%u imu=%u hid=%u mouse=%u waitRelease=%u buttonIdle=%u pinchRaw=%u pinchStable=%u pairKey=%u pairWindow=%u enabled=%u releasePending=%u\n",
      unsigned(allow),unsigned(imuReady),unsigned(ready),unsigned(mouseMode),unsigned(waitRelease),
      unsigned(button==Button::Idle),unsigned(rawLast),unsigned(stableState),unsigned(digitalRead(PairButton::Pin)),
      unsigned(pairWindow()),unsigned(gesturesEnabled),unsigned(releasePending));
  }
  if(!allow) Gesture::reset();
  bool mouseActive=mouseMode && ready && !recording && !waitRelease && !pairWindow();
  bool mouseButtonBusy=rawLast==LOW || stableState==LOW || button!=Button::Idle ||
    releasePending || digitalRead(PairButton::Pin)==LOW;
  if(mouseButtonBusy) mouseBlockedAt=now;
  bool mouseBlocked=mouseButtonBusy || now-mouseBlockedAt<100;
  if(!mouseActive || !imuReady || now-lastSample>100) TiltMouse::reset();
  if(imuReady && now-lastSample>=SAMPLE_MS){
    lastSample=now;
    Wire.beginTransmission(imuAddress);Wire.write(0x3B);
    bool valid=Wire.endTransmission(false)==0;
    uint8_t bytes[14];
    if(valid){valid=Wire.requestFrom(imuAddress,uint8_t(14))==14;}
    if(valid){for(auto& b:bytes)b=Wire.read();}
    else {while(Wire.available())Wire.read();Gesture::reset();TiltMouse::reset();}
    if((!valid || millis()-now>100) && gestureDebug && !recording && now-lastReadErrorLog>=1000){
      lastReadErrorLog=now;
      Serial.printf("# GST IMU_READ_ERROR valid=%u read_ms=%lu address=0x%02X\n",unsigned(valid),(unsigned long)(millis()-now),imuAddress);
    }
    if(valid && millis()-now<=100){
      auto readAxis=[&](int i){return int16_t((uint16_t(bytes[i])<<8)|bytes[i+1]);};
      // Preserve configured +/-8g and +/-2000 deg/s ranges.
      Gesture::Sample sample{readAxis(0)*9.80665f/4096,readAxis(2)*9.80665f/4096,readAxis(4)*9.80665f/4096,
        readAxis(8)*0.0174532925f/16.4f,readAxis(10)*0.0174532925f/16.4f,readAxis(12)*0.0174532925f/16.4f};
      if(recording && recordActive && stableState==LOW)
        Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",(unsigned long)now,
          sample.ax,sample.ay,sample.az,sample.gx,sample.gy,sample.gz);
      auto delta=TiltMouse::update(sample,now,mouseActive && linkEpoch.load()==seenEpoch && linkUsable(),mouseBlocked,gestureDebug && !recording);
      if((delta.x || delta.y) && !releasePending && linkEpoch.load()==seenEpoch && linkUsable()){
        uint8_t report[3]={0,uint8_t(delta.x),uint8_t(delta.y)};
        if(!mouseReport->notify(report,sizeof(report))){
          // Drop failed relative movement; no backlog or delayed cursor jump.
          TiltMouse::reset();
          if(gestureDebug) Serial.println("# MOUSE SEND FAILED: movement dropped");
        }
      }
      int event=Gesture::update(sample,now,allow && linkEpoch.load()==seenEpoch && linkUsable());
      if(event && gestureDebug && !recording) Serial.printf("# GST EVENT %s hid_release_busy=%u\n",event>0?"NEXT":"PREVIOUS",unsigned(releasePending));
      if(event && !releasePending){
        uint8_t k[8]={0,0,uint8_t(event>0?0x4E:0x4B),0,0,0,0,0};
        bool sent=keyboardReport->notify(k,sizeof(k));releasePending=true;pressedAt=now;
        Serial.println(sent?(event>0?"# GESTURE NEXT":"# GESTURE PREVIOUS"):"# HID_PRESS_FAILED");
      }
    } else if(valid){Gesture::reset();TiltMouse::reset();}
  }
  delay(1);
}
