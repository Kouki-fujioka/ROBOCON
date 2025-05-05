import processing.serial.*;
Serial port1; // 1台目のZumoのシリアル通信用
Serial port2; // 2台目のZumoのシリアル通信用
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
String mode_str;
boolean sof_f = false; // SoF(Start of flame) を発見したかどうかのフラグ

void setup()
{
  size(1200, 800);                        // 幅 1200px, 高さ 800px のウインドウを生成
  port1 = new Serial(this, "COM4", 9600); // Serial クラスのインスタンスを生成

   port2 = new Serial(this, "COM5", 9600); // Serial クラスのインスタンスを生成

  background(0); // 背景色を黒に

  fill(255, 255, 255);
  rect(0, 0, width / 2, height); // 左の領域を塗りつぶす
  fill(255, 255, 255);
  rect(width / 2, 0, width / 2, height); // 右の領域を塗りつぶす
  textSize(50);                          // 文字の大きさを50
  PFont font = createFont("Meiryo", 50);
  textFont(font);
}

void draw()
{
  if (zumo_id == 1)
  { // データを受信したときだけ書き換える（1番目のZumo）
    fill(255, 255, 255);
    rect(0, 0, width / 2, height); // 対象画面の初期化

    // スピードメーターの描画
    stroke(0, 0, 0);                                  // 線の色を黒
    ellipse(150, 300, 200, 200);                       // (150,300)を中心,幅200,高さ200の円
    ellipse(450, 300, 200, 200);                      // (450,300)を中心,幅200,高さ200の円
    stroke(255, 0, 0);                                // 線の色を赤
    line(150, 300, 150 + 100 * y_l, 300 - 100 * x_l);   // (150,200),(150+100*y_l,200-100*x_l)を結ぶ線
    line(450, 300, 450 + 100 * y_r, 300 - 100 * x_r); // (350,200),(350+100*y_r,200-100*x_r)を結ぶ線

    // 色グラフの描画
    stroke(0, 0, 0);                                  // 線の色を黒
    fill(red, 0, 0);
    rect(10, height, 100, -red);
    fill(0, green, 0);
    rect(120, height, 100, -green);
    fill(0, 0, blue);
    rect(230, height, 100, -blue);
    fill(red, green, blue);
    rect(350, height, 200, -200);

    // 文字の描画
    fill(0, 0, 0);
    text("状態:" + mode_str, 10, 50);
    text("距離:" + dist, 10, 100);
  }

  if (zumo_id == 2)
  { // データを受信したときだけ書き換える（2番目のZumo）
    fill(255, 255, 255);
    rect(width / 2, 0, width / 2, height); // 対象画面の初期化

    // スピードメーターの描画
    stroke(0, 0, 0);                                  // 線の色を黒
    ellipse(750, 300, 200, 200);                      // (750,300)を中心,幅200,高さ200の円
    ellipse(1050, 300, 200, 200);                      // (1050,300)を中心,幅200,高さ200の円
    stroke(255, 0, 0);                                // 線の色を赤
    line(750, 300, 750 + 100 * y_l, 300 - 100 * x_l); // (750,300),(750+100*y_l,300-100*x_l)を結ぶ線
    line(1050, 300, 1050 + 100 * y_r, 300 - 100 * x_r); // (1050,300),(1050+100*y_r,300-100*x_r)を結ぶ線

    // 色グラフの描画
    stroke(0, 0, 0);                                  // 線の色を黒
    fill(red, 0, 0);
    rect(610, height, 100, -red);
    fill(0, green, 0);
    rect(720, height, 100, -green);
    fill(0, 0, blue);
    rect(830, height, 100, -blue);
    fill(red, green, blue);
    rect(950, height, 200, -200);

    // 文字の描画
    fill(0, 0, 0);
    text("状態:" + mode_str, 610, 50);
    text("距離:" + dist, 610, 100);
  }
}

// 通信方式2
void serialEvent(Serial p)
{
  if ((p == port1 || p == port2) && (p.available() > 0))
  { // 割り込みシリアル通信が，port1か，port2で，なおかつデータが入っている時
    if (p == port1)
      zumo_id = 1; // データをやり取りしたロボットのIDを記憶
    else if (p == port2)
      zumo_id = 2; // データをやり取りしたロボットのIDを記憶
  }
  int l = p.available(); // 受信バッファ内のデータ数
  boolean bod_f = false; // 1組のデータ(block of data)が得られたか？

  while (l > 0)
  { // 受信バッファ内にデータがある場合
    if (sof_f == false)
    { // SoFを発見していない場合
      if (p.read() == 'H')
      {               // SoF(Start of Frame)の検査
        sof_f = true; // SoFの発見
      }
      l--; // 受信バッファのデータ数の修正
    }
    if (sof_f == true)
    { // SoFを発見している場合
      if (l >= 7)
      { // 受信バッファのデータ数が7以上
        red = p.read();
        green = p.read();
        blue = p.read();
        mode = p.read();
        dist = p.read();
        motor_l = p.read();
        motor_r = p.read();

        omega_l = (motor_l / 250) * 2 * PI; // ラジアンに変換(円周率=PI)
        x_l = cos(omega_l);                 // -1~1の値に変換
        y_l = sin(omega_l);                 // -1~1の値に変換

        omega_r = (motor_r / 250) * 2 * PI; // ラジアンに変換(円周率=PI)
        x_r = cos(omega_r);                 // -1~1の値に変換
        y_r = sin(omega_r);                 // -1~1の値に変換

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
          mode_str = "キャッチ前の直進";
          break;
        case 7:
          mode_str = "キャッチ後の直進";
          break;
        case 8:
          mode_str = "衝突回避";
          break;
        case 9:
          mode_str = "初期状態直後;";
          break;
        case 10:
          mode_str = "衝突回避後の直進;";
          break;
        }

        bod_f = true;  // 1組のデータを読み込んだ
        sof_f = false; // 1組のデータを読み取ったのでSoFの発見をクリア
        l -= 7;       // 受信バッファのデータ数の修正
      } else
      {        // 受信バッファのデータ数不足の場合
        break; // whileループを中断
      }
    }
  }
  if (bod_f == true) // 1組のデータを読み込んだので
    p.write("A");    // 次のデータ送信要求を送信
}
