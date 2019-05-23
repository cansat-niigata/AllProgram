/*---------------------------------------------
 * GPSの情報をパソコンに表示するための関数
 --------------------------------------------*/
void PrintGPSInfo(){
  Serial1.print(gps.satellites.value());
  Serial1.print(",     ");
  Serial1.print(gps.location.lat(),6);
  Serial1.print(",  ");
  Serial1.print(gps.location.lng(),6);
  Serial1.print(",  ");
  char sz[32];
  sprintf(sz, "%02d/%02d/%02d ", gps.date.year(), gps.date.month(), gps.date.day());
  Serial1.print(sz);
  Serial1.print(",");
  sprintf(sz, "%02d:%02d:%02d ", gps.time.hour()+9, gps.time.minute(), gps.time.second());
  Serial1.print(sz);
  Serial1.print(",");
  Serial1.print(gps.course.deg(),3);
  Serial1.print(", ");
  Serial1.print(gps.speed.kmph(),1);
  Serial1.print(", ");
  Serial1.print(TinyGPSPlus::cardinal(gps.course.deg()));
}
