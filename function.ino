// function

int cup_cnt = 0;

// 超音波センサによる距離の測定
int distance() {
  unsigned long interval;  // Echoのパルス幅(μs)
  int dst;                 // 距離(cm)

  // 10μsのパルスを超音波センサのTrigピンに出力
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // 5767μs(100cm)、超音波が反射してこなければタイムアウトしてエラー値0を返す
  // Echo信号がHIGHである時間(μs)を計測
  interval = pulseIn(echo, HIGH, 5767);

  dst = (0.61 * 25 + 331.5) * interval / 10000 / 2;  // 距離(cm)に変換
  if (dst == 0) {
    dst = 100;  // エラー値0（超音波が反射してこない）は100cmを検出したことにする
  }
  delay(60);  // trigがHIGHになる間隔を60ms以上空ける（超音波センサの仕様）
  return dst;
}

// カップがロボットのアームの中に入っているか識別 (AFTERCATCH内で使用)
void check_cupstate() {
  if (dist <= 5 && dist != 0) {  // アームの中にカップが入っている時
    cup_cnt = 0;                 // カウントリセット
    catch_flag = 1;              // キャッチ済
  } else {                       // アームの中にカップが入っていない時
    cup_cnt++;                   // 値を1増やす
    if (cup_cnt == 5) {          // 値が5の時(5回連続入っていないと検知した時),カップを見失ったと判断
      cup_cnt = 0;               // カウントリセット
      catch_flag = 0;            // キャッチ前に戻す
      mode_G = SEARCH;
      start_time_G = timeNow_G;
    }
  }
}

// メインタスク
void task() {
  if (button.isPressed() == true) {
    if (stop_flag == 1) {
      mode_G = INIT;
    } else {
      mode_G = STOP;
    }
  } else {
    switch (mode_G) {
      case INIT:  // 初期状態
        mode_G = AFTER_INIT;
        start_time_G = timeNow_G;
        stop_flag = 0;
        catch_flag = 0;
        goal_flag = 0;
        avoidance_flag = 0;
        break;

      case AFTER_INIT:  // 初期状態直後専用
        stop_flag = 0;
        if (myteam == RED_TEAM) {                 // 自チームが赤チーム
          if (timeNow_G - start_time_G < 2000) {  // 2.0秒間,コートの相手の方向に前進
            speed = 250;
            diff = turnTo(21);

            if (dist < 40 && dist != 0) {  // 前進中に距離が40cm未満の値を計測すると物体を検知
              mode_G = DETECTION;
              start_time_G = timeNow_G;
              detected_dist = dist;
            }
          } else {  // 2.0秒後探索に移る
            mode_G = SEARCH;
            start_time_G = timeNow_G;
          }
        } else {                                  // 自チームが青チーム
          if (timeNow_G - start_time_G < 2000) {  // 2.0秒間,コートの相手の方向に前進
            speed = 250;
            diff = turnTo(200);

            if (dist < 40 && dist != 0) {  // 前進中に距離が40cm未満の値を計測すると物体を検知
              mode_G = DETECTION;
              start_time_G = timeNow_G;
              detected_dist = dist;
            }
          } else {  // 2.0秒後探索に移る
            mode_G = SEARCH;
            start_time_G = timeNow_G;
          }
        }
        break;

      case FORWARD:  // 直進
        stop_flag = 0;
        avoidance_flag = 0;
        // if (ax < -40 && ay < -40) {
        //   mode_G = AVOIDANCE;
        //   start_time_G = timeNow_G;
        // }
        if (timeNow_G - start_time_G < 2500) {  // 2.5秒前進
          speed = 200;
          diff = 0;
          if (dist < 40 && dist != 0) {  // 前進中に距離が40cm未満の値を計測すると物体を検知
            mode_G = DETECTION;
            start_time_G = timeNow_G;
            detected_dist = dist;
          }
        } else  // 2.5秒前進後は探索を行う
        {
          mode_G = SEARCH;
          start_time_G = timeNow_G;
        }
        break;

      case SEARCH:  // 探索
        stop_flag = 0;
        // if (ax < -40 && ay < -40) {
        //   mode_G = AVOIDANCE;
        //   start_time_G = timeNow_G;
        // }
        if (timeNow_G - start_time_G < 3500) {  // 3.5秒間回転
          speed = 0;
          diff = 150;

          if (dist < 40 && dist != 0) {  // 探索中に距離が40cm未満の値を計測すると物体を検知
            mode_G = DETECTION;
            start_time_G = timeNow_G;
            detected_dist = dist;
          }
        } else {  // 何も検知できなかった（半径15cm以内に物体がない）場合は前進
          mode_G = FORWARD;
          start_time_G = timeNow_G;
        }
        break;

      case ROTATE:  // 回転
        stop_flag = 0;
        if (goal_flag == 1) {                    // 自チームのゴールラインを踏んでいる (ゴール時)
          if (timeNow_G - start_time_G < 300) {  // 0.3秒後退
            speed = -150;
            diff = 0;
          } else if (timeNow_G - start_time_G < 1100) {  // 0.8秒回転
            speed = 0;
            diff = 200;
          } else  // 回転後は直進
          {
            goal_flag = 0;
            mode_G = FORWARD;
            start_time_G = timeNow_G;
          }
        } else {                                   // それ以外の白色以外のラインを踏んだ時
          if (catch_flag == 0) {                   // カップを持っていない時
            if (timeNow_G - start_time_G < 300) {  // 0.3秒後退
              speed = -150;
              diff = 0;
            } else if (timeNow_G - start_time_G < 1400) {  // 1.1秒回転
              speed = 0;
              diff = 150;

              // 回転中の検知に近くに物体があれば検知
              if (timeNow_G - start_time_G > 800) {
                if (dist < 40 && dist != 0) {  // 探索中に距離が30cm未満の値を計測すると物体を検知
                  mode_G = DETECTION;
                  start_time_G = timeNow_G;
                  detected_dist = dist;
                }
              }
            } else {
              mode_G = FORWARD;
              start_time_G = timeNow_G;
            }
          } else {                                    // カップを持っている時
            if (myteam == RED_TEAM) {                 // 赤チームの時
              if (timeNow_G - start_time_G < 1000) {  // 1.0秒回転
                speed = 270;
                diff = 230;
                color_ignore = 1;                            // 一時的に色を検知しても通常の回転をしないようにする
              } else if (timeNow_G - start_time_G < 1300) {  // 0.20秒の間に色を検知する
                if (identify_color(160, 56, 24)) {           // 赤を検知した場合
                  is_detected = RED;
                } else if (identify_color(22, 31, 0)) {  // 黒を検知した場合
                  is_detected = BLACK;
                } else {  // その他(白) 黒or赤踏むまで前進
                  speed = 250;
                  diff = 0;
                }
              } else if (timeNow_G - start_time_G < 2000) {  // 0.8秒間,検知した色によって異なる動作を行う
                if (is_detected == RED) {                    // 赤 :地磁気で角度変更しながら前進
                  speed = 200;
                  diff = turnTo(350);               // 少し角度を付ける
                } else if (is_detected == BLACK) {  // 黒 ：前進
                  speed = 250;
                  diff = turnTo(70);
                }
              } else {  // 2.0秒後
                mode_G = AFTERCATCH;
                color_ignore = 0;
                start_time_G = timeNow_G;
              }
            } else {                                  // 青チームの時
              if (timeNow_G - start_time_G < 1000) {  // 1.0秒回転
                speed = 270;
                diff = 230;
                color_ignore = 1;                            // 一時的に色を検知しても通常の回転をしないようにする
              } else if (timeNow_G - start_time_G < 1300) {  // 0.20秒の間に色を検知する
                if (identify_color(37, 72, 103)) {           // 青を検知した場合
                  is_detected = BLUE;
                } else if (identify_color(22, 31, 0)) {  // 黒を検知した場合
                  is_detected = BLACK;
                } else {  // その他(白)黒or青踏むまで前進
                  speed = 200;
                  diff = 0;
                }
              } else if (timeNow_G - start_time_G < 2000) {  // 0.8秒間,検知した色によって異なる動作を行う
                if (is_detected == BLUE) {                   // 青 :地磁気で角度変更しながら前進
                  speed = 200;
                  diff = turnTo(160);               // 少し角度をつける
                } else if (is_detected == BLACK) {  // 黒 ：前進
                  speed = 250;
                  diff = turnTo(240);
                }
              } else {  // 2.0秒後
                mode_G = AFTERCATCH;
                color_ignore = 0;  // 回転検知できるように戻す
                start_time_G = timeNow_G;
              }
            }
          }
        }
        break;

      case DETECTION:  // 検知したものを判定
        stop_flag = 0;
        diff = 0;
        // if (ax < -40 && ay < -40) {
        //   mode_G = AVOIDANCE;
        //   start_time_G = timeNow_G;
        // }

        // 2.距離と時間を測定して固定値で算出する場合
        if (timeNow_G - start_time_G <= 100) {  // 0.1秒間微調整
          speed = 0;
          diff = -150;
          detected_dist = dist;
        } else if (timeNow_G - start_time_G <= 300) {  // 0.2秒間進む
          speed = 100;
        } else if (timeNow_G - start_time_G > 300) {  // それ以降
          if (detected_dist - dist <= 8.0) {          // 物体をカップと判定する
            mode_G = GETFORWARD;
            start_time_G = timeNow_G;
          } else {  // 物体をロボットと判定する
            mode_G = AVOIDANCE;
            start_time_G = timeNow_G;
          }
        }
        break;

      case STOP:  // 停止
        stop_flag = 1;
        speed = 0;
        diff = 0;
        break;

      case GETFORWARD:  // 直進（キャッチ前）
        stop_flag = 0;
        speed = 150;
        diff = 0;
        // if (ax < -40 && ay < -40) {
        //   mode_G = AVOIDANCE;
        //   start_time_G = timeNow_G;
        // }
        if (dist <= 4 && dist != 0) {  // 距離が4cm以下になった時
          mode_G = AFTERCATCH;
          start_time_G = timeNow_G;
        } else if (dist == 100) {  // 途中で物体を見失ったら、探索に戻る
          mode_G = SEARCH;
          start_time_G = timeNow_G;
        }
        break;

      case AFTERCATCH:  // 回転+直進（キャッチ後）
        stop_flag = 0;
        check_cupstate();  // カップがアームの中に入っているかを確認

        if (myteam == RED_TEAM && catch_flag == 1) {  // 自チームが赤
          if (abs(200 - heading_G) <= 10) {           // 赤ゴールの方向に回転してから前進
            speed = 250;
            diff = turnTo(200);
          } else {  // 赤ゴールの方向に回転
            speed = 150;
            diff = turnTo(200);
            // 2秒以内に自ゴールの方向に向かなかったら(上のif文を満たさない場合)、物体をロボットとみなす
            if (timeNow_G - start_time_G >= 2000) {
              mode_G = AVOIDANCE;
              start_time_G = timeNow_G;
            }
          }
        } else if (myteam == BLUE_TEAM && catch_flag == 1) {  // 自チームが青
          if (abs(21 - heading_G) <= 10) {                    // 青ゴールの方向に回転してから前進
            speed = 250;
            diff = turnTo(21);
          } else {  // 青ゴールの方向に回転
            speed = 150;
            diff = turnTo(21);
            // 2秒以内に自ゴールの方向に向かなかったら(上のif文を満たさない場合)、物体をロボットとみなす
            if (timeNow_G - start_time_G >= 2000) {
              mode_G = AVOIDANCE;
              start_time_G = timeNow_G;
            }
          }
        }
        break;

      case AVOIDANCE:  // ロボットとの衝突回避
        stop_flag = 0;
        buzzer.play(">c32");
        avoidance_flag = 1;
        if (timeNow_G - start_time_G < 1000) {  // 1.0秒後退
          speed = -150;
          diff = 0;
        } else if (timeNow_G - start_time_G < 1300) {  // 0.3秒回転
          speed = 0;
          diff = 150;
        } else {
          mode_G = FORWARD;
          start_time_G = timeNow_G;
        }

        if (identify_color(22, 31, 0) || identify_color(160, 56, 24) || identify_color(37, 72, 103)) {  // 黒か赤か青の線を踏んだ場合は直進
          mode_G = AVOIDANCEFORWARD;
          start_time_G = timeNow_G;
        }
        break;

      case AVOIDANCEFORWARD:
        stop_flag = 0;
        avoidance_flag = 0;
        if (timeNow_G - start_time_G < 500) {  // 0.5秒前進
          color_ignore = 1;
          speed = 200;
          diff = 0;
        } else  // 0.5秒前進後は探索を行う
        {
          color_ignore = 0;
          mode_G = SEARCH;
          start_time_G = timeNow_G;
        }
        break;
    }

    motorL_G = speed + diff;
    motorR_G = speed - diff;

    compass.read();  // 加速度センサ・地磁気センサの値を読み取る
    // 各センサの各軸の値がそれまでの最大値よりも大きいか、最小値よりも小さいとき値を書き換える
    compass.m_min.x = min(compass.m.x, compass.m_min.x);
    compass.m_max.x = max(compass.m.x, compass.m_max.x);
    compass.m_min.y = min(compass.m.y, compass.m_min.y);
    compass.m_max.y = max(compass.m.y, compass.m_max.y);
    compass.m_min.z = min(compass.m.z, compass.m_min.z);
    compass.m_max.z = max(compass.m.z, compass.m_max.z);
    mx = map(compass.m.x, compass.m_min.x, compass.m_max.x, -128, 127);
    my = map(compass.m.y, compass.m_min.y, compass.m_max.y, -128, 127);
    mz = map(compass.m.z, compass.m_min.z, compass.m_max.z, -128, 127);
    ax = compass.a.x / 256;
    ay = compass.a.x / 256;
    az = compass.a.x / 256;

    sum_e = 0.0;

    // 黒や相手ゴールの色を踏んだ時の処理 (主にロボットの回転)

    if (color_ignore == 0 && avoidance_flag == 0) {  // color_ignoreが1の時は別の処理を行う (ROTATEにて)
      if (catch_flag == 0)                           // キャッチ前
      {
        if (identify_color(22, 31, 0) || identify_color(160, 56, 24) || identify_color(37, 72, 103)) {  // 黒か赤か青の線を踏んだ場合は回転
          mode_G = ROTATE;
          start_time_G = timeNow_G;
        }
      } else  // キャッチ後
      {
        if (myteam == RED_TEAM) {                                          // 自チームが赤
          if (identify_color(22, 31, 0) || identify_color(37, 72, 103)) {  // 黒か青の線を踏んだ場合は回転
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
          if (identify_color(160, 56, 24)) {  // 赤の線を踏んだ場合はゴール
            goal_flag = 1;
            catch_flag = 0;
            cup_cnt = 0;
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
        } else {                                                           // 自チームが青
          if (identify_color(22, 31, 0) || identify_color(160, 56, 24)) {  // 黒か赤の線を踏んだ場合は回転
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
          if (identify_color(37, 72, 103)) {  // 青の線を踏んだ場合はゴール
            goal_flag = 1;
            catch_flag = 0;
            cup_cnt = 0;
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
        }
      }
    }
  }
}

int maintainState(unsigned long period) {
  static int flagStart = 0;  // 0:待ち状態，1:現在計測中
  static unsigned long startTime = 0;

  if (flagStart == 0) {
    startTime = timeNow_G;  // 計測を開始したtimeNow_Gの値を覚えておく
    flagStart = 1;          // 現在計測中にしておく
  }

  if (timeNow_G - startTime > period) {  // 計測開始からの経過時間が指定時間を越えた
    flagStart = 0;                       // 待ち状態に戻しておく
    startTime = 0;                       // なくても良いが，形式的に初期化
    return 1;
  } else
    return 0;
}
