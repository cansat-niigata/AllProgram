//移動の制御を行う関数
/*-------------------------------------------------------
 *  目的地方向と現在の進行方向がなす角を求める関数
 * ------------------------------------------------------
 *  引数  goal       : 目的地の方向(double)
 *        myLocation : 進行方向(double)
 *  戻値  angular    :目的地と進行方向の角度(double)
 *  機能  : 目的地の方向と進行方向がなす角を求める
 *  　　　　　進行方向に対して目的地が右にあるときは0～180
 *  　　　　　進行方向に対して目的地が左にあるときは0～-180
 *  　　　　の値を返す
 --------------------------------------------------------*/
double angularDifference(double goal, double myLocation){
  double angular = goal - myLocation;
  //angularの値域が-180～180に収まるように値を変換
  if(angular > 180.0)
    angular -= 360;
  if(angular < -180)
    angular += 360;
  return angular;
}
