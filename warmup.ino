/*-----------------------------------------------
 *  GPSが同じ値をとるようになるまで待機する関数
 ------------------------------------------------*/
/*
void warmingupGPS() {
  const int DATANUM = 3;
  double lat[DATANUM] = {}, lng[DATANUM] = {};
  bool valid = false;
  bool endFlag = false;
  int i = 0;
  while (!endFlag) {
    while (ss.available() > 0) {
      gps.encode(ss.read());
    }

    if (gps.location.isUpdated()) {
      lat[i] = gps.location.lat();
      lng[i] = gps.location.lng();
      for (int j = 0; j < DATANUM - 1; j++) {
        if (lat[j] != lat[j + 1] || lng[j] != lng[j + 1]) {
          valid = true;
          break;
        }
      }
      if (valid) {
        Serial.println("MISS");
        i++;
        i = i % DATANUM;
        valid = false;
      } else {
        endFlag = true;
        Serial.println("Warm Up is Done");
      }
    }
  }
}
*/
