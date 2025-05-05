// main

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
#define RED_TEAM 0   // 自チームが赤チーム
#define BLUE_TEAM 1  // 自チームが青チーム

// 色
#define RED 3
#define BLUE 4
#define BLACK 5


// ロボットの状態
#define INIT 0        // 初期状態
#define FORWARD 1     // 直進
#define ROTATE 2      // 回転
#define STOP 3        // 停止
#define SEARCH 4      // 探索
#define DETECTION 5   // 検知
#define GETFORWARD 6  // 直進（キャッチ前）
#define AFTERCATCH 7  // 直進（キャッチ後）
#define AVOIDANCE 8   // ロボットとの衝突回避
#define AFTER_INIT 9  // 初期状態直後
#define AVOIDANCEFORWARD 10 // ロボットとの衝突回避


// カラーセンサ,システム,時間等のglobal変数
float red_G, green_G, blue_G;                                              // カラーセンサで読み取った現在のRGB値（0-255）
float speed;                                                               // モーターの直進速度
float diff;                                                                // モーターの回転速度
int motorR_G, motorL_G;                                                    // 左右のZumoのモータに与える回転力
int myteam;                                                                // チームカラー
int mode_G;                                                                // タスクのモードを表す状態変数
int catch_flag;                                                            // キャッチしたかどうかを表すフラグ
int goal_flag;                                                             // ゴール線を通過したかどうかを表すフラグ
int avoidance_flag;                                                        // 衝突回避中に色を踏んだ場合のフラグ
int stop_flag;                                                        // 停止フラグ
unsigned long timeInit_G, timeNow_G, start_time_G, timePrev_G, ax_time_G;  // スタート時間,経過時間,時間計測の開始時間
int color_ignore = 0;                                                      // (ignore = 1の時) 一時的に色を検知しても回転しないようにする
int is_detected = 0;                                                       // 検知した色を格納


// 超音波センサのglobal変数
const int trig = 2;       // Trigピン
const int buzzerPin = 3;  // Buzzerピン
const int echo = 4;       // Echoピン
int dist;                 // 距離(cm)
int detected_dist;        // 検知したときの距離(cm)
int cnt = 0;              // カウント

// 地磁気センサのglobal変数
float ax = 0, ay = 0, az = 0;
float mx = 0, my = 0, mz = 0;
float heading_G = 0;
float ax_dist = 0, ax_sum = 0;
float sum_e = 0;



void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupCompass();

  button.waitForButton();    // Zumo buttonが押されるまで待機
  calibrationCompass();      // 地磁気センサのキャリブレーション
  button.waitForButton();    // Zumo buttonが押されるまで待機
  CalibrationColorSensor();  // カラーセンサーのキャリブレーション
  button.waitForButton();    // Zumo buttonが押されるまで待機

  timeInit_G = millis();

  pinMode(trig, OUTPUT);  // trigを出力ポートに設定
  pinMode(echo, INPUT);   // echoを入力ポートに設定

  mode_G = INIT;
  speed = 0;
  diff = 0;
  start_time_G = 0;
  timePrev_G = 0;
  catch_flag = 0;
  goal_flag = 0;

  // 自チームが赤の時
  myteam = RED_TEAM;
  // 自チームが青の時
  // myteam = BLUE_TEAM;
}

void loop() {
  getRGB(red_G, green_G, blue_G);        // カラーセンサでRGB値を取得（0-255）
  timeNow_G = millis() - timeInit_G;     // 経過時間
  motors.setSpeeds(motorL_G, motorR_G);  // 左右モーターへの回転力入力
  dist = distance();                     // 距離を計測

  sendData();  // データ送信

  task();  // メインタスク

  timePrev_G = timeNow_G;  // 1回前の時刻に現在時刻を代入
}

// 通信方式2
void sendData() {
  static unsigned long timePrev = 0;
  static boolean flag_start = true;  // 最初の通信かどうか
  int inByte;

  // if文の条件： 最初の通信である || 最後のデータ送信から500ms経過 || (データ送信要求が来ている && 最後のデータ送信から50ms経過)
  if (flag_start == true || timeNow_G - timePrev > 500 || (Serial.available() > 0 && timeNow_G - timePrev > 50)) {
    flag_start = false;
    while (Serial.available() > 0) {  // 送信要求が複数来ていた場合は全て読み込む
      inByte = Serial.read();
    }

    Serial.write('H');
    Serial.write((int)red_G);
    Serial.write((int)green_G);
    Serial.write((int)blue_G);
    Serial.write(mode_G);
    Serial.write(dist);
    Serial.write(motorL_G);
    Serial.write(motorR_G);

    timePrev = timeNow_G;
  }
}
