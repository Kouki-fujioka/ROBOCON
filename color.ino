#include <Adafruit_TCS34725.h> // カラーセンサライブラリ

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X); // カラーセンサオブジェクト

unsigned int r_min, g_min, b_min; // RGB 最小値
unsigned int r_max, g_max, b_max; // RGB 最大値

// キャリブレーション (ロボット → 色紙 (黒色 ~ 白色) 通過)
void CalibrationColorSensor()
{
  unsigned long timeInit;    // キャリブレーション開始時刻
  unsigned int r, g, b, clr; // RGB, 明度

  tcs.begin();              // カラーセンサ初期化
  motors.setSpeeds(60, 60); // 前進

  // 初期化 (更新用)
  r_min = 30000;
  g_min = 30000;
  b_min = 30000;
  r_max = 0;
  g_max = 0;
  b_max = 0;

  timeInit = millis(); // キャリブレーション開始時刻

  while (1)
  {
    tcs.getRawData(&r, &g, &b, &clr); // RGB, 明度取得

    // RGB 最小値更新
    if (r < r_min)
      r_min = r;
    if (g < g_min)
      g_min = g;
    if (b < b_min)
      b_min = b;

    // RGB 最大値更新
    if (r > r_max)
      r_max = r;
    if (g > g_max)
      g_max = g;
    if (b > b_max)
      b_max = b;

    if (millis() - timeInit > 2000) // 2 秒間キャリブレーション実行
      break;
  }

  motors.setSpeeds(0, 0); // 停止
}

// RGB (スケーリング済) 取得
void getRGB(float &r0, float &g0, float &b0)
{
  unsigned int r, g, b, clr; // RGB, 明度

  tcs.getRawData(&r, &g, &b, &clr); // RGB, 明度取得

  // スケーリング
  r0 = map(r, r_min, r_max, 0, 255);
  g0 = map(g, g_min, g_max, 0, 255);
  b0 = map(b, b_min, b_max, 0, 255);

  if (r0 < 0.0)
    r0 = 0.0;
  if (r0 > 255.0)
    r0 = 255.0;
  if (g0 < 0.0)
    g0 = 0.0;
  if (g0 > 255.0)
    g0 = 255.0;
  if (b0 < 0.0)
    b0 = 0.0;
  if (b0 > 255.0)
    b0 = 255.0;
}

// 色識別
int identify_color(int red, int green, int blue)
{
  float d2;          // 色距離 (色空間上)
  float d2_max = 50; // 閾値（適宜調整）

  d2 = pow(red - red_G, 2) + pow(green - green_G, 2) + pow(blue - blue_G, 2);

  if (d2 < d2_max * d2_max)
    return 1;
  else
    return 0;
}
