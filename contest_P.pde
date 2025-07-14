import processing.serial.*;

Serial port1; // Zumo シリアル通信用 (1 台目)
Serial port2; // Zumo シリアル通信用 (2 台目)

int zumo_id = 0;
int mode;
int green, red, blue;
int dist;
float motor_l;
float motor_r;
float omega_l;
float omega_r;
float x_l;
float x_r;
float y_l;
float y_r;
float mx, my, mz;
String mode_str;       // 現在モード
boolean sof_f = false; // SoF (Start of flame) 発見フラグ

void setup()
{
  size(1200, 800);                        // ウィンドウ生成 (1200 * 800)
  port1 = new Serial(this, "COM4", 9600); // COM4 接続
  port2 = new Serial(this, "COM5", 9600); // COM5 接続
  background(0);                          // 背景色設定 (黒)
  fill(255, 255, 255);                    // 塗色設定 (白)
  rect(0, 0, width / 2, height);          // 白色四角形描画 (左領域)
  rect(width / 2, 0, width / 2, height);  // 白色四角形描画 (右領域)
  textSize(50);                           // 文字サイズ設定
  PFont font = createFont("Meiryo", 50);  // フォントオブジェクト作成
  textFont(font);                         // フォント設定
}

void draw()
{
  if (zumo_id == 1) // 1 台目
  {
    fill(255, 255, 255);           // 塗色設定 (白)
    rect(0, 0, width / 2, height); // 対象画面初期化

    // スピードメータ描画
    stroke(0, 0, 0);                                  // 線色設定 (黒)
    ellipse(150, 300, 200, 200);                      // 円描画 (中心座標 (150, 300), 横幅 200, 縦幅 200)
    ellipse(450, 300, 200, 200);                      // 円描画 (中心座標 (450, 300), 横幅 200, 縦幅 200)
    stroke(255, 0, 0);                                // 線色設定 (赤)
    line(150, 300, 150 + 100 * y_l, 300 - 100 * x_l); // 線描画 ((150, 200) ~ (150 + 100 * y_l, 200-100 * x_l))
    line(450, 300, 450 + 100 * y_r, 300 - 100 * x_r); // 線描画 ((350, 200) ~ (350 + 100 * y_r, 200-100 * x_r))

    // 検知色グラフ描画
    stroke(0, 0, 0);                // 線色設定 (黒)
    fill(red, 0, 0);                // 塗色設定 (R)
    rect(10, height, 100, -red);    // R 色四角形描画
    fill(0, green, 0);              // 塗色設定 (G)
    rect(120, height, 100, -green); // G 色四角形描画
    fill(0, 0, blue);               // 塗色設定 (B)
    rect(230, height, 100, -blue);  // B 色四角形描画
    fill(red, green, blue);         // 塗色設定 (RGB)
    rect(350, height, 200, -200);   // RGB 色四角形描画

    // 文字描画
    fill(0, 0, 0);                      // 文字色設定 (黒)
    text("状態 : " + mode_str, 10, 50); // テキスト描画
    text("距離 : " + dist, 10, 100);    // テキスト描画
  }

  if (zumo_id == 2) // 2 台目
  {
    fill(255, 255, 255);                   // 塗色設定 (白)
    rect(width / 2, 0, width / 2, height); // 対象画面初期化

    // スピードメータ描画
    stroke(0, 0, 0);                                    // 線色設定 (黒)
    ellipse(750, 300, 200, 200);                        // 円描画 (中心座標 (750, 300), 横幅 200, 縦幅 200)
    ellipse(1050, 300, 200, 200);                       // 円描画 (中心座標 (1050, 300), 横幅 200, 縦幅 200)
    stroke(255, 0, 0);                                  // 線色設定 (赤)
    line(750, 300, 750 + 100 * y_l, 300 - 100 * x_l);   // 線描画 ((750, 300) ~ (750 + 100 * y_l, 300 - 100 * x_l))
    line(1050, 300, 1050 + 100 * y_r, 300 - 100 * x_r); // 線描画 ((1050, 300) ~ (1050 + 100 * y_r, 300 - 100 * x_r))

    // 検知色グラフ描画
    stroke(0, 0, 0);                // 線色設定 (黒)
    fill(red, 0, 0);                // 塗色設定 (R)
    rect(610, height, 100, -red);   // R 色四角形描画
    fill(0, green, 0);              // 塗色設定 (G)
    rect(720, height, 100, -green); // G 色四角形描画
    fill(0, 0, blue);               // 塗色設定 (B)
    rect(830, height, 100, -blue);  // B 色四角形描画
    fill(red, green, blue);         // 塗色設定 (RGB)
    rect(950, height, 200, -200);   // RGB 色四角形描画

    // 文字描画
    fill(0, 0, 0);                     // 文字色設定 (黒)
    text("状態:" + mode_str, 610, 50); // テキスト描画
    text("距離:" + dist, 610, 100);    // テキスト描画
  }
}

// 通信方式 2
void serialEvent(Serial p)
{
  if ((p == port1 || p == port2) && (p.available() > 0)) // (割り込みシリアル通信 == port1 or port2) && 受信データ有
  {
    if (p == port1)
      zumo_id = 1; // ロボット ID 記憶
    else if (p == port2)
      zumo_id = 2; // ロボット ID 記憶
  }

  int l = p.available(); // 受信データ数
  boolean bod_f = false; // データ 1 組読み込み前

  while (l > 0) // 受信データ有
  {
    if (sof_f == false) // SoF 未発見
    {
      if (p.read() == 'H') // SoF 検査
      {
        sof_f = true; // SoF 発見
      }
      l--; // 受信データ数更新
    }
    if (sof_f == true) // SoF 発見
    {
      if (l >= 7) // 受信データ数 7 以上
      {
        red = p.read();     // R
        green = p.read();   // G
        blue = p.read();    // B
        mode = p.read();    // モード
        dist = p.read();    // 検知距離
        motor_l = p.read(); // 左モータ回転力
        motor_r = p.read(); // 右モータ回転力

        omega_l = (motor_l / 250) * 2 * PI; // ラジアン変換 (円周率 = PI)
        x_l = cos(omega_l);                 // 値変換 (-1 ~ 1)
        y_l = sin(omega_l);                 // 値変換 (-1 ~ 1)

        omega_r = (motor_r / 250) * 2 * PI; // ラジアン変換 (円周率 = PI)
        x_r = cos(omega_r);                 // 値変換 (-1 ~ 1)
        y_r = sin(omega_r);                 // 値変換 (-1 ~ 1)

        switch (mode)
        {
        case 0:
          mode_str = "初期状態";
          break;
        case 1:
          mode_str = "直進";
          break;
        case 2:
          mode_str = "回転";
          break;
        case 3:
          mode_str = "停止";
          break;
        case 4:
          mode_str = "探索";
          break;
        case 5:
          mode_str = "検知";
          break;
        case 6:
          mode_str = "キャッチ前直進";
          break;
        case 7:
          mode_str = "キャッチ後直進";
          break;
        case 8:
          mode_str = "衝突回避";
          break;
        case 9:
          mode_str = "初期状態直後";
          break;
        case 10:
          mode_str = "衝突回避後直進";
          break;
        }

        bod_f = true;  // データ 1 組読み込み後
        sof_f = false; // SoF 発見フラグクリア
        l -= 7;        // 受信データ数更新
      }
      else // 受信データ数不足
      {
        break;
      }
    }
  }

  if (bod_f == true) // データ 1 組読み込み後
    p.write("A");    // 次データ送信要求送信
}
