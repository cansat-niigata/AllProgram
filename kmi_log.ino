/*------------------------------------
 * kmllog書き込みの準備用関数
 -------------------------------------*/
/*
void PrintKML_header(){
  kml_log_file.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
  kml_log_file.println("<kml xmlns=\"http://earth.google.com/kml/2.2\">");
  kml_log_file.println("<Placemark>");
  kml_log_file.println("<name>Runback-runnninng-log</name>");
  kml_log_file.println("<description>Runback-runnning-log-in-NIIGATA</description>");
  kml_log_file.println("<LineString>");
  kml_log_file.println("<coordinates>");
  kml_log_file.flush();
}

void SaveToSDcardWithKML(){
  kml_log_file.print("");
  kml_log_file.print(gps.location.lng(), 8);
  kml_log_file.print(",");
  kml_log_file.print(gps.location.lat(), 8);
  kml_log_file.print(",");
  kml_log_file.println(gps.altitude.meters());
  kml_log_file.flush();
}

void CloseKML(){
  kml_log_file.println("</coordinates>");
  kml_log_file.println("</LineString>");
  kml_log_file.println("</Placemark>");
  kml_log_file.println("</kml>");
  kml_log_file.close();
}
*/
