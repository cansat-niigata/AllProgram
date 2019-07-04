/*----------------------------------------*/
/* 走行プログラム(stm32f103)　　             */
/*----------------------------------------*/
#include <SD.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <UseMotorSTM32f103.h>
#include <PID.h>

TinyGPSPlus gps;

//SD----------------------------------------
File datafile; //csv形式
#define SD_PIN PA4 //SPI通信に使うピン
void SaveToSDcard();
//------------------------------------------

//Servo-------------------------------------
Servo myservo;
#define servoPin PA3 //9番ピン
void detouchParachute(int PinNum);
//------------------------------------------

//フライトピン------------------------------
#define FlightPin PA0
//------------------------------------------

//LED-----------------------------
#define LED_R PC14
#define LED_L PC15
void Lchika(int times);
void turnOffTheLight();
//GPS------------------------------------------------
//static const double GOAL_LAT = 37.866434, GOAL_LON = 138.938515;//目的地の位置座標（コウガクブトウマエ）
//static const double GOAL_LAT = 37.867500, GOAL_LON = 138.937146;//目的地の位置座標（ブシツトウワキ）
static const double GOAL_LAT = 37.871406, GOAL_LON = 138.942138;//目的地の位置座標（ヤキュウジョウ）

double PRE_LAT = 0, PRE_LON = 0; //以前の位置情報
bool start_flag = false;
double delta = 0;
const int x = 15; //delta*xが機体の旋回時間
void PrintGPSInfo();
unsigned long distanceKmToGoal;
double courseToGoal;
//-----------------------------------------------------------------------------------

//モータ用の変数--------------------------------------------
#define rPWMPin PB8
#define rPin PB9
#define lPWMPin PB7
#define lPin PB6
UseMotor motor_r(rPWMPin, rPin);
UseMotor motor_l(lPWMPin, lPin);
double angularDifference(double goal, double myLocation);
//----------------------------------------------------------

//エンコーダ用変数----------------------------------------
volatile int r = 0, l = 0; //割り込み時に使用する関数の変数
void changePWM();
#define WARIKOMI_R PA1
#define WARIKOMI_L PA2
//---------------------------------------------------------


//PID制御関連--------------------------------------------------
PID pid_r, pid_l;
const int target1 = 80;
const int target2 = 70;
void changePWM();
void stackMove();
//--------------------------------------------------------------

//割り込み用関数--------------------------------------------------
void R() {
  r++;
}
void L() {
  l++;
}
//----------------------------------------------------------------

//その他--------------------------------------------------------
float meter = 1.0;
float close = 5.0;
//---------------------------------------------------------------

void setup() {
  //ピンモードの設定------------
  pinMode(rPWMPin, OUTPUT);
  pinMode(rPin, OUTPUT);
  pinMode(lPWMPin, OUTPUT);
  pinMode(lPin, OUTPUT);
  pinMode(FlightPin, INPUT);
  pinMode(PC14, OUTPUT);
  pinMode(PC15, OUTPUT);


  //PID制御の設定---------------
  pid_r.set_Target(target1);
  pid_r.set_Kp(0.2);
  pid_r.set_conVar(180);
  pid_l.set_Target(target1);
  pid_l.set_Kp(0.2);
  pid_l.set_conVar(180);

  //割り込みの設定
  Timer1.pause();
  Timer1.setPrescaleFactor(10000);
  Timer1.setOverflow(1800); //0.5秒でタイマー割り込み
  Timer1.setCompare(1, 1800);
  Timer1.attachInterrupt(1, changePWM);
  Timer1.refresh();
  Timer1.pause();

  Serial1.begin(9600);

  //-------------SDの設定-----------------
  pinMode(SD_PIN, OUTPUT); //SDに使うピンを設定

  if (!SD.begin(SD_PIN)) {
    Serial1.println(F("Card failed, or not present"));
  } else {
    Serial1.println(F("Card initialized."));
    datafile = SD.open("log.csv", FILE_WRITE);
  }
  digitalWrite(SD_PIN, HIGH); //SDの回線を遮断
  //---------------------------------------

  //割り込みの設定
  attachInterrupt(digitalPinToInterrupt(WARIKOMI_R), R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WARIKOMI_L), L, CHANGE);
  Lchika(3);
}

void loop() {
  int flightPin = 100;
  LchikaWait();
  while (flightPin > 50) {
    flightPin = analogRead(FlightPin);
    Serial1.println(flightPin);
    delay(10);
  }
  turnOffTheLight();
  countDown(30); //30秒待機する
  //パラシュートを切り離す
  detouchParachute(servoPin);
  LchikaStraight();
  //まずはパラシュートから離れるために直進する(10s)
  motor_r.rotationNormal(pid_r.get_conVar());
  motor_l.rotationNormal(pid_l.get_conVar());

  Timer1.resume();
  delay(10000);
  motor_r.stop();
  motor_l.stop();
  Timer1.refresh();
  Timer1.pause();
  turnOffTheLight();

  while (1) {
    //GPSの値を取得する
    //値が更新されるまで待機
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }

    //GPSの情報をSerial通信でパソコンに送信
    if (gps.location.isUpdated()) {
      //------------------GPS情報の表示--------------------
      Lchika(5);
      turnOffTheLight();
      PrintGPSInfo();
      distanceKmToGoal = (unsigned long)TinyGPSPlus::distanceBetween(
                           gps.location.lat(),
                           gps.location.lng(),
                           GOAL_LAT,
                           GOAL_LON);
      Serial1.print(distanceKmToGoal);
      Serial1.print(",  ");

      courseToGoal = TinyGPSPlus::courseTo(
                       gps.location.lat(),
                       gps.location.lng(),
                       GOAL_LAT,
                       GOAL_LON);
      Serial1.print(courseToGoal);
      Serial1.print(",  ");
      const char *cardinalToGoal = TinyGPSPlus::cardinal(courseToGoal);
      Serial1.print(cardinalToGoal);
      Serial1.print(",  ");

      double direction = TinyGPSPlus::courseTo(
                           PRE_LAT,
                           PRE_LON,
                           gps.location.lat(),
                           gps.location.lng());
      //-------------------------------------------------------------
      //-----------------制御に使用する位置情報の更新----------------
      PRE_LAT = gps.location.lat();
      PRE_LON = gps.location.lng();
      //同時に目的地までの距離、進行方向と目的地の方向の角度差θを計算
      //θは-180～180の間になるように調整
      delta = angularDifference(courseToGoal, direction);  //ゴール方向と移動方向のなす角
      Serial1.println(delta);
      SaveToSDcard();
      Timer1.pause();
      //目的地までの距離がゴール判定の基準に達したか？
      //目的地までの距離が十分近ければゴールしたと判断する
      if (distanceKmToGoal < meter) {
        //制御終了
        motor_r.stop();
        motor_l.stop();
        Serial1.println(F("GOAL!!"));
        datafile.println();
        datafile.println(F("GOAL"));
        datafile.close();
        while (true) {
          digitalWrite(PC14, HIGH);
          digitalWrite(PC15, HIGH);
          delay(500);
          digitalWrite(PC14, LOW);
          digitalWrite(PC15, LOW);
          delay(500);
        }
      } else {
        if (distanceKmToGoal < close) {
          pid_r.set_Target(target2);
          pid_l.set_Target(target2);
        }
        if (delta > 0) {       //目的地から離れていた場合
          //deltaの値が正の時
          //左タイヤをdeltaに比例した量だけ回転
          motor_r.stop();
          motor_l.stop();
          LchikaLeft();
          motor_l.rotationNormal(pid_l.get_conVar());
          delay(x * delta);
        } else {
          //θの値が負の時
          //右タイヤをdeltaに比例した量だけ回転
          delta *= -1;
          motor_r.stop();
          motor_l.stop();
          LchikaLeft();
          motor_r.rotationNormal(pid_r.get_conVar());
          delay(x * delta);
        }
        //直進(次のGPSの値取得まで行う)
        //この時PID制御を行う
        //タイマー割り込みを利用
        r = 0; l = 0;
        LchikaStraight();
        motor_r.rotationNormal(pid_r.get_conVar());
        motor_l.rotationNormal(pid_l.get_conVar());
        Timer1.refresh();
        Timer1.resume();
      }
    }
  }
}
