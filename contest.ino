#include <Wire.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <LSM303.h>
#include <ZumoBuzzer.h>

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;
ZumoBuzzer buzzer;

// チーム
#define RED_TEAM 0  // 自チーム → 赤
#define BLUE_TEAM 1 // 自チーム → 青

// 色
#define RED 3
#define BLUE 4
#define BLACK 5

// ロボット状態
#define INIT 0              // 初期状態
#define FORWARD 1           // 直進
#define ROTATE 2            // 回転
#define STOP 3              // 停止
#define SEARCH 4            // 探索
#define DETECTION 5         // 検知
#define GETFORWARD 6        // 直進（キャッチ前）
#define AFTERCATCH 7        // 直進（キャッチ後）
#define AVOIDANCE 8         // ロボット衝突回避
#define AFTER_INIT 9        // 初期状態直後
#define AVOIDANCEFORWARD 10 // ロボット衝突回避

// カラーセンサ, モータ
float red_G, green_G, blue_G; // RGB
float speed;                  // モータ直進速度
float diff;                   // モータ回転速度
int motorR_G, motorL_G;       // 左右モータ回転力

int myteam;           // チームカラー
int mode_G;           // モード
int catch_flag;       // カップキャッチフラグ
int goal_flag;        // ゴール線通過フラグ
int avoidance_flag;   // 色検知 (衝突検知中) フラグ
int stop_flag;        // 停止フラグ
int color_ignore = 0; // ignore == 1 : 色検知 → 回転無効
int is_detected = 0;  // 色 (検知)
unsigned long timeInit_G, timeNow_G, start_time_G, timePrev_G, ax_time_G;

// 超音波センサ
const int trig = 2;      // Trig ピン
const int buzzerPin = 3; // Buzzer ピン
const int echo = 4;      // Echo ピン
int dist;                // 現在検知距離 (cm)
int detected_dist;       // 前回検知距離 (cm)
int cnt = 0;             // カウント

// 地磁気センサ
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;
float heading_G = 0;
float ax_dist = 0, ax_sum = 0;
float sum_e = 0;

void setup()
{
  Serial.begin(9600); // シリアル通信開始 (9600bps)
  Wire.begin();       // I2C 通信開始
  setupCompass();     // コンパス設定

  button.waitForButton(); // Zumo button 押下待機
  calibrationCompass();   // 地磁気センサキャリブレーション

  button.waitForButton();   // Zumo button 押下待機
  CalibrationColorSensor(); // カラーセンサキャリブレーション

  button.waitForButton(); // Zumo button 押下待機
  timeInit_G = millis();  // 開始時刻
  pinMode(trig, OUTPUT);  // trig → 出力ポート
  pinMode(echo, INPUT);   // echo → 入力ポート

  // 初期状態
  mode_G = INIT;
  speed = 0;
  diff = 0;
  start_time_G = 0;
  timePrev_G = 0;
  catch_flag = 0;
  goal_flag = 0;

  // チームカラー選択
  // myteam = RED_TEAM;
  myteam = BLUE_TEAM;
}

void loop()
{
  getRGB(red_G, green_G, blue_G);       // RGB (スケーリング済) 格納
  timeNow_G = millis() - timeInit_G;    // 経過時間
  motors.setSpeeds(motorL_G, motorR_G); // 左右モーター回転
  dist = distance();                    // 距離 (ロボット → 障害物)
  sendData();                           // データ送信
  task();                               // メインタスク
  timePrev_G = timeNow_G;               // 時刻更新
}

// 通信方式 2
void sendData()
{
  static unsigned long timePrev = 0; // 前回データ送信時刻
  static boolean flag_start = true;  // 初期通信
  int inByte;                        // 受信データ格納用

  // 初期通信 || 前回データ送信後 500ms 経過 || (受信データ (送信要求) 有 && 前回データ送信後 50ms 経過)
  if (flag_start == true || timeNow_G - timePrev > 500 || (Serial.available() > 0 && timeNow_G - timePrev > 50))
  {
    flag_start = false;

    while (Serial.available() > 0) // 受信データ (送信要求) 有
    {
      inByte = Serial.read(); // 全受信データ (送信要求) クリア (Serial.available() = 0)
    }

    // データ送信 (Processing 描画用)
    Serial.write('H');          // ヘッダ
    Serial.write((int)red_G);   // R
    Serial.write((int)green_G); // G
    Serial.write((int)blue_G);  // B
    Serial.write(mode_G);       // モード
    Serial.write(dist);         // 現在検知距離 (ロボット → 障害物)
    Serial.write(motorL_G);     // 左モータ回転力
    Serial.write(motorR_G);     // 右モータ回転力

    timePrev = timeNow_G; // 前回データ送信時刻更新
  }
}
