//SD関連の関数

/*-------------------------------------------------------------------
 * SDに制御記録を書き込むための関数
 * ------------------------------------------------------------------
 * 引数・戻り値  : なし
 * 機能         : GPSで取得したデータと、進行方向の制御をSDに書き込む
 ------------------------------------------------------------------*/
 
void SaveToSDcard(){
  datafile.println();
  datafile.print(gps.satellites.value()); datafile.print(",");
  datafile.print(gps.hdop.hdop()); datafile.print(",");
  datafile.print(gps.location.lat(), 6); datafile.print(",");
  datafile.print(gps.location.lng(), 6); datafile.print(",");
  datafile.print(gps.location.age()); datafile.print(",");
  datafile.print(gps.date.month()); datafile.print(",");
  datafile.print(gps.date.day()); datafile.print(",");
  datafile.print(gps.date.year()); datafile.print(",");
  datafile.print(gps.time.hour() + 9); datafile.print(",");//日本時間に変換
  datafile.print(gps.time.minute()); datafile.print(",");
  datafile.print(gps.time.second()); datafile.print(",");
  datafile.print(gps.altitude.meters()); datafile.print(",");
  datafile.print(gps.course.deg()); datafile.print(",");
  datafile.print(gps.speed.kmph()); datafile.print(",");
  datafile.print(TinyGPSPlus::cardinal(gps.course.deg())); datafile.print(",");
  datafile.print(distanceKmToGoal); datafile.print(",");
  datafile.print(courseToGoal); datafile.print(",");
  datafile.print(delta); datafile.print(",");
  datafile.flush();
}
