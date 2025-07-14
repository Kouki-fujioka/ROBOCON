int cup_cnt = 0; // 紛失回数

// 距離 (ロボット → 障害物) 測定 (超音波センサ)
int distance()
{
  unsigned long interval; // Echo パルス幅 (μs)
  int dst;                // 検知距離 (cm)

  // 超音波パルス発射 (10 マイクロ秒間)
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  // 超音波パルス停止
  digitalWrite(trig, LOW);

  interval = pulseIn(echo, HIGH, 5767); // 超音波往復時間 (5767 μs ≒ 100 cm)

  // 検知距離 (cm) = 音速 (m/s) × 時間 (s) ÷ 2 × 100
  dst = (0.61 * 25 + 331.5) * interval / 10000 / 2; // 検知距離 (cm)

  if (dst == 0) // 超音波非反射 (計測不可能)
  {
    dst = 100;
  }

  delay(60); // 超音波パルス発射間隔 >= 60ms (超音波センサ仕様)

  return dst;
}

// 判定 (カップ → アーム中)
void check_cupstate()
{
  if (dist <= 5 && dist != 0) // カップ → アーム中
  {
    cup_cnt = 0;    // カウントリセット
    catch_flag = 1; // キャッチ済
  }
  else // カップ → アーム外
  {
    cup_cnt++;

    if (cup_cnt == 5)
    {
      cup_cnt = 0; // カウントリセット
      catch_flag = 0;
      mode_G = SEARCH;
      start_time_G = timeNow_G;
    }
  }
}

// メインタスク
void task()
{
  if (button.isPressed() == true) // Zumo button 押下
  {
    if (stop_flag == 1)
    {
      mode_G = INIT;
    }
    else
    {
      mode_G = STOP;
    }
  }
  else
  {
    switch (mode_G)
    {
    case INIT: // 初期状態
      mode_G = AFTER_INIT;
      start_time_G = timeNow_G;
      stop_flag = 0;
      catch_flag = 0;
      goal_flag = 0;
      avoidance_flag = 0;
      break;

    case AFTER_INIT: // 初期状態直後専用
      stop_flag = 0;

      if (myteam == RED_TEAM) // 赤チーム
      {
        if (timeNow_G - start_time_G < 2000) // 2 秒間
        {
          // 前進 (相手コート)
          speed = 250;
          diff = turnTo(21);

          if (dist < 40 && dist != 0) // 障害物検知 (40 cm 未満)
          {
            mode_G = DETECTION;
            start_time_G = timeNow_G;
            detected_dist = dist;
          }
        }
        else // 2 秒経過後
        {
          mode_G = SEARCH;
          start_time_G = timeNow_G;
        }
      }
      else // 青チーム
      {
        if (timeNow_G - start_time_G < 2000) // 2 秒間
        {
          // 前進 (相手コート)
          speed = 250;
          diff = turnTo(200);

          if (dist < 40 && dist != 0) // 障害物検知 (40 cm 未満)
          {
            mode_G = DETECTION;
            start_time_G = timeNow_G;
            detected_dist = dist;
          }
        }
        else // 2 秒経過後
        {
          mode_G = SEARCH;
          start_time_G = timeNow_G;
        }
      }
      break;

    case FORWARD: // 直進
      stop_flag = 0;
      avoidance_flag = 0;

      if (timeNow_G - start_time_G < 2500) // 2.5 秒間
      {
        // 前進
        speed = 200;
        diff = 0;

        if (dist < 40 && dist != 0) // 障害物検知 (40 cm 未満)
        {
          mode_G = DETECTION;
          start_time_G = timeNow_G;
          detected_dist = dist;
        }
      }
      else // 2.5 秒経過後
      {
        mode_G = SEARCH;
        start_time_G = timeNow_G;
      }
      break;

    case SEARCH: // 探索
      stop_flag = 0;

      if (timeNow_G - start_time_G < 3500) // 3.5 秒間
      {
        // 回転
        speed = 0;
        diff = 150;

        if (dist < 40 && dist != 0) // 障害物検知 (40 cm 未満)
        {
          mode_G = DETECTION;
          start_time_G = timeNow_G;
          detected_dist = dist;
        }
      }
      else // 非検知
      {
        mode_G = FORWARD;
        start_time_G = timeNow_G;
      }
      break;

    case ROTATE: // 回転
      stop_flag = 0;

      if (goal_flag == 1) // ゴール時
      {
        if (timeNow_G - start_time_G < 300) // 0.3 秒間
        {
          // 後退
          speed = -150;
          diff = 0;
        }
        else if (timeNow_G - start_time_G < 1100) // 0.8 秒間
        {
          // 回転
          speed = 0;
          diff = 200;
        }
        else // 回転後
        {
          goal_flag = 0;
          mode_G = FORWARD;
          start_time_G = timeNow_G;
        }
      }
      else
      {
        if (catch_flag == 0) // カップ非所持
        {
          if (timeNow_G - start_time_G < 300) // 0.3 秒間
          {
            // 後退
            speed = -150;
            diff = 0;
          }
          else if (timeNow_G - start_time_G < 1400) // 1.1 秒間
          {
            // 回転
            speed = 0;
            diff = 150;

            if (timeNow_G - start_time_G > 800) // 0.5 秒経過後
            {
              if (dist < 40 && dist != 0) // 障害物検知 (40 cm 未満)
              {
                mode_G = DETECTION;
                start_time_G = timeNow_G;
                detected_dist = dist;
              }
            }
          }
          else // 1.1 秒経過後
          {
            mode_G = FORWARD;
            start_time_G = timeNow_G;
          }
        }
        else // カップ所持
        {
          if (myteam == RED_TEAM) // 赤チーム
          {
            if (timeNow_G - start_time_G < 1000) // 1 秒間
            {
              // 前進回転
              speed = 270;
              diff = 230;
              color_ignore = 1; // 色検知無視 (非回転)
            }
            else if (timeNow_G - start_time_G < 1300) // 0.3 秒間
            {
              if (identify_color(160, 56, 24)) // 赤検知
              {
                is_detected = RED;
              }
              else if (identify_color(22, 31, 0)) // 黒検知
              {
                is_detected = BLACK;
              }
              else // 白検知
              {
                // 前進
                speed = 250;
                diff = 0;
              }
            }
            else if (timeNow_G - start_time_G < 2000) // 0.7 秒間
            {
              if (is_detected == RED) // 赤検知
              {
                // 前進
                speed = 200;
                diff = turnTo(350); // 角度調整
              }
              else if (is_detected == BLACK) // 黒検知
              {
                // 前進
                speed = 250;
                diff = turnTo(70); // 角度調整
              }
            }
            else // 2 秒経過後
            {
              mode_G = AFTERCATCH;
              color_ignore = 0; // 色検知再開
              start_time_G = timeNow_G;
            }
          }
          else // 青チーム
          {
            if (timeNow_G - start_time_G < 1000) // 1 秒間
            {
              // 前進回転
              speed = 270;
              diff = 230;
              color_ignore = 1; // 色検知無視 (非回転)
            }
            else if (timeNow_G - start_time_G < 1300) // 0.3 秒間
            {
              if (identify_color(37, 72, 103)) // 青検知
              {
                is_detected = BLUE;
              }
              else if (identify_color(22, 31, 0)) // 黒検知
              {
                is_detected = BLACK;
              }
              else // 白検知
              {
                // 前進
                speed = 200;
                diff = 0;
              }
            }
            else if (timeNow_G - start_time_G < 2000) // 0.7 秒間
            {
              if (is_detected == BLUE) // 青検知
              {
                // 前進
                speed = 200;
                diff = turnTo(160); // 角度調整
              }
              else if (is_detected == BLACK) // 黒検知
              {
                // 前進
                speed = 250;
                diff = turnTo(240); // 角度調整
              }
            }
            else // 2 秒経過後
            {
              mode_G = AFTERCATCH;
              color_ignore = 0; // 色検知再開
              start_time_G = timeNow_G;
            }
          }
        }
      }
      break;

    case DETECTION: // 障害物判定 (カップ or ロボット)
      stop_flag = 0;
      diff = 0;

      if (timeNow_G - start_time_G <= 100) // 0.1 秒間
      {
        // 回転
        speed = 0;
        diff = -150;
        detected_dist = dist;
      }
      else if (timeNow_G - start_time_G <= 300) // 0.2 秒間
      {
        // 前進
        speed = 100;
      }
      else if (timeNow_G - start_time_G > 300) // 0.3 秒経過後
      {
        if (detected_dist - dist <= 8.0) // 検知距離差 8cm 以下
        {
          // 障害物 == カップ
          mode_G = GETFORWARD;
          start_time_G = timeNow_G;
        }
        else
        {
          // 障害物 == ロボット
          mode_G = AVOIDANCE;
          start_time_G = timeNow_G;
        }
      }
      break;

    case STOP: // 停止
      stop_flag = 1;
      speed = 0;
      diff = 0;
      break;

    case GETFORWARD: // 直進 (キャッチ前)
      stop_flag = 0;
      speed = 150;
      diff = 0;

      if (dist <= 4 && dist != 0) // 検知距離 4cm 以下
      {
        mode_G = AFTERCATCH;
        start_time_G = timeNow_G;
      }
      else if (dist == 100) // 紛失
      {
        mode_G = SEARCH;
        start_time_G = timeNow_G;
      }
      break;

    case AFTERCATCH: // 回転 + 直進 (キャッチ後)
      stop_flag = 0;
      check_cupstate(); // 判定 (カップ → アーム中)

      if (myteam == RED_TEAM && catch_flag == 1) // 赤チーム
      {
        if (abs(200 - heading_G) <= 10) // ゴール方向回転済
        {
          // 前進 (ゴール方向)
          speed = 250;
          diff = turnTo(200); // 角度調整
        }
        else
        {
          // 回転 (ゴール方向)
          speed = 150;
          diff = turnTo(200); // 角度調整

          if (timeNow_G - start_time_G >= 2000) // ゴール方向回転 2 秒以上
          {
            // 障害物 == ロボット
            mode_G = AVOIDANCE;
            start_time_G = timeNow_G;
          }
        }
      }
      else if (myteam == BLUE_TEAM && catch_flag == 1) // 青チーム
      {
        if (abs(21 - heading_G) <= 10) // ゴール方向回転済
        {
          // 前進 (ゴール方向)
          speed = 250;
          diff = turnTo(21); // 角度調整
        }
        else
        {
          // 回転 (ゴール方向)
          speed = 150;
          diff = turnTo(21); // 角度調整

          if (timeNow_G - start_time_G >= 2000) // ゴール方向回転 2 秒以上
          {
            // 障害物 == ロボット
            mode_G = AVOIDANCE;
            start_time_G = timeNow_G;
          }
        }
      }
      break;

    case AVOIDANCE: // ロボット同士衝突回避
      stop_flag = 0;
      buzzer.play(">c32"); // ブザー
      avoidance_flag = 1;

      if (timeNow_G - start_time_G < 1000) // 1 秒間
      {
        // 後退
        speed = -150;
        diff = 0;
      }
      else if (timeNow_G - start_time_G < 1300) // 0.3 秒間
      {
        // 回転
        speed = 0;
        diff = 150;
      }
      else // 回転後
      {
        mode_G = FORWARD;
        start_time_G = timeNow_G;
      }

      if (identify_color(22, 31, 0) || identify_color(160, 56, 24) || identify_color(37, 72, 103)) // 検知 (黒 or 赤 or 青)
      {
        mode_G = AVOIDANCEFORWARD;
        start_time_G = timeNow_G;
      }
      break;

    case AVOIDANCEFORWARD:
      stop_flag = 0;
      avoidance_flag = 0;

      if (timeNow_G - start_time_G < 500) // 0.5 秒間
      {
        // 前進
        color_ignore = 1; // 色検知無視 (非回転)
        speed = 200;
        diff = 0;
      }
      else // 前進後
      {
        color_ignore = 0; // 色検知再開
        mode_G = SEARCH;
        start_time_G = timeNow_G;
      }
      break;
    }

    motorL_G = speed + diff; // 左モータ回転力
    motorR_G = speed - diff; // 右モータ回転力

    compass.read(); // データ取得 (加速度, 地磁気)

    // 地磁気最小値更新
    compass.m_min.x = min(compass.m.x, compass.m_min.x);
    compass.m_min.y = min(compass.m.y, compass.m_min.y);
    compass.m_min.z = min(compass.m.z, compass.m_min.z);

    // 地磁気最大値更新
    compass.m_max.x = max(compass.m.x, compass.m_max.x);
    compass.m_max.y = max(compass.m.y, compass.m_max.y);
    compass.m_max.z = max(compass.m.z, compass.m_max.z);

    // 地磁気スケーリング
    mx = map(compass.m.x, compass.m_min.x, compass.m_max.x, -128, 127);
    my = map(compass.m.y, compass.m_min.y, compass.m_max.y, -128, 127);
    mz = map(compass.m.z, compass.m_min.z, compass.m_max.z, -128, 127);

    // 加速度スケーリング
    ax = compass.a.x / 256;
    ay = compass.a.y / 256;
    az = compass.a.z / 256;

    sum_e = 0.0; // 角度差積分値

    if (color_ignore == 0 && avoidance_flag == 0)
    {
      if (catch_flag == 0) // キャッチ前
      {
        if (identify_color(22, 31, 0) || identify_color(160, 56, 24) || identify_color(37, 72, 103)) // 検知 (黒 or 赤 or 青)
        {
          mode_G = ROTATE;
          start_time_G = timeNow_G;
        }
      }
      else // キャッチ後
      {
        if (myteam == RED_TEAM) // 赤チーム
        {
          if (identify_color(22, 31, 0) || identify_color(37, 72, 103)) // 検知 (黒 or 青)
          {
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
          if (identify_color(160, 56, 24)) // 赤検知
          {
            // ゴール
            goal_flag = 1;
            catch_flag = 0;
            cup_cnt = 0;
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
        }
        else // 青チーム
        {
          if (identify_color(22, 31, 0) || identify_color(160, 56, 24)) // 検知 (黒 or 赤)
          {
            mode_G = ROTATE;
            start_time_G = timeNow_G;
          }
          if (identify_color(37, 72, 103)) // 青検知
          {
            // ゴール
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
