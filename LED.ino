/*----------------------------------
   LEDを消す
  -----------------------------------*/
void turnOffTheLight() {
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_L, LOW);
}
/*-------------------------------
   LEDを点滅させるプログラム
   ------------------------------
   回数を引数で指定する
  --------------------------------*/
void Lchika(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_L, HIGH);
    delay(200);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_L, LOW);
    delay(200);
  }
}

/*-------------------------------
   待機状態を表すLED点灯パターン
  --------------------------------*/
void LchikaWait() {
  digitalWrite(LED_R, HIGH);
}

/*--------------------------------
   カウントダウンするときの点滅
  --------------------------------
   引数でカウントダウンの秒数を指定
  ---------------------------------*/
void countDown(int time) {
  for (int i = 0; i < (time - 10); i++) { //30カウント
    if ((i % 2) == 0) {
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, LOW);
    } else {
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, HIGH);
    }
    delay(1000);
  }
  for (int i = 0; i < 20; i++) {
    if ((i % 2) == 0) {
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_L, LOW);
    } else {
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_L, HIGH);
    }
    delay(500);
  }
}

/*-----------------------
   直進時の点滅パターン
  -----------------------*/
void LchikaStraight() {
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_L, HIGH);
}

/*----------------------------
 * 右旋回時の点滅パターン
 -----------------------------*/
 void LchikaRight(){
  digitalWrite(LED_R, HIGH);
 }

/*--------------------------------
 * 左旋回時の点滅パターン
 --------------------------------*/
 void LchikaLeft(){
  digitalWrite(LED_L, HIGH);
 }
