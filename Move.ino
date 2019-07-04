/*--------------------------------------------------------
   割り込みに使う関数
  ---------------------------------------------------------*/
void changePWM() {
  pid_r.update_conVar(r);
  pid_l.update_conVar(l);
  Serial1.print(r); Serial1.print(F(", ")); Serial1.println(l);
  r = 0; l = 0;
  motor_r.rotationNormal(pid_r.get_conVar());
  motor_l.rotationNormal(pid_l.get_conVar());
}

/*----------------------------------------
 *  スタック判定時の挙動(その場で旋回)
----------------------------------------- */
void stackMove(){
  motor_r.stop();
  motor_l.stop();
  motor_r.rotationNormal();
  motor_l.rotationInvert();
  delay(1000);
  motor_r.rotationInvert();
  motor_l.rotationNormal();
  delay(1000);
  motor_r.stop();
  motor_l.stop();
}

/*--------------------------------------------------------
 * サーボモータによるパラシュート切り離しの動作
 * プログラムの中でServo.hのインクルードが必要
 ---------------------------------------------------------*/
 void detouchParachute(int PinNum){
  Servo myservo;
  Serial1.println(F("Open the Cover"));
  myservo.attach(PinNum);
  myservo.write(0);
  delay(2000); //TinyGPS++を使わない場合 smartDelay→delay に変更してください
  myservo.write(180);
  delay(2000);
  myservo.write(0);
  delay(2000);
  myservo.write(180);
  delay(2000);
  myservo.write(0);
  delay(2000);
  myservo.write(180);
  delay(2000);
  myservo.write(0);
  myservo.detach();
  Serial1.println(F("Finish!"));
 }
