/* Happy_RC_Receiver(version 6b) *****************************
  Download a Transmitter App:https://github.com/TomokiIkegami/Happy_RC_Driver/raw/main/Happy_RC_Driver.apk
  About this Project:https://github.com/TomokiIkegami/Happy_RC_Driver

  ◆ 動作
  1.スマホアプリ(Happy_RC_Receiver)から命令（'A'～'I'）を取得
  2.取得した命令に基づいて、RCを前後左右に操作
  ◆ 機能
  ・アプリを開いていないときは、車体を停止させる安全機能を装備（Dragブレーキ付きESCを使用すること）
  ・アプリ再起動時には、走行停止ボタン（画面中央のPボタン）を押さないと走行しない（アプリ再起動時に急に走行する危険を防ぐため）
  ・カクカクではなく滑らかな走行が可能に
  ◆ 補足
  ・プログラム中で★マークがついている部分は、自分の装置に合わせて調整
*************************************************************/

/*ハードウェアの接続ピンの設定*/
#define LED_PIN 2 //走行には無関係。2番ピンにLEDのアノード(+)を接続すると割り込み処理の間隔(100ms)でLEDが点滅
#define LED_PIN_ACTIVE 17 //走行には無関係。17番ピンにLEDのアノード(+)を接続すると、アプリ起動時のみLEDが点灯
#define SERVO_PWM_PIN 4 //サーボモータのPWMピン（信号入力ピン）をESP32の4番ピンに接続 ★回路と対応した番号にする
#define ESC_PWM_PIN 16 //ESCのPWMピン（信号入力ピン）をESP32の16番ピンに接続 ★回路と対応した番号にする

/*タイマーの定義*/
hw_timer_t*timer = NULL;

/*ライブラリ*/
#include "BluetoothSerial.h" //ESP32のBluetooth通信に使用
#include <Servo.h>　//サーボモータの制御に使用

/*ESC,サーボのオブジェクト作成*/
Servo myservo;    // サーボモータを制御するためのServoオブジェクト作成
Servo myesc;    // ESCを制御するためのServoオブジェクト作成

/* ステアリングの設定 */
unsigned long mov_speed_ST = 40; //ステア移動速度
int center_pos = 93; //ステア中心位置 [サーボモータの中心位置 (90°)]　★まっすぐ走るように調整。90より大きい値にするとステア（ハンドル）が右寄りに、90より小さい値にするとステア（ハンドル）が左寄りになる
int left_DR = 20; //左の切れ角 ★:好みに合わせて調整。ただし大きくしすぎないように注意。
int right_DR = 25; //右の切れ角 ★:好みに合わせて調整。ただし大きくしすぎないように注意。
int left_max = center_pos - left_DR; //左ステアの最大位置 [中心位置より反時計回りに20°（left_DR）回転した位置] ★逆に動くときはleft_DRの手前の符号をプラス（+）に
int right_max = center_pos + right_DR; //右ステアの最大位置 [中心位置より時計回りに25°（right_DR）回転した位置] ★逆に動くときはright_DRの手前の符号をマイナス（-）に

/* スロットルの設定 */
unsigned long mov_speed_TH = 0; //スロットル移動速度
unsigned long mov_speed_brk = 40; //ブレーキ速度
int neutral_pos = 91; //中立位置 [スロットルの中立位置 (90) ★ESCの設定によってずれがあるので、前後に走行しないよう値を調整する。※ ESC側を90で中立になるよう設定（上級者向け。ESCの説明書通りプロポでニュートラル設定を済ませてから、このプログラムの値を調整するのがオススメ）してもよい。]
int forward_DR = 20; //前進の速さ ★好みの速度に調整
int backward_DR = 20; //バックの速さ ★好みの速度に調整
int forward_max = neutral_pos + forward_DR; //前進の最大位置 ★逆に動くときはforward_DRの手前の符号をマイナス（-）に
int backward_max = neutral_pos - backward_DR; //バックの最大位置 ★逆に動くときはbackward_DRの手前の符号をプラス（+）に

/*値設定の注意点*/
//速度(mov_speed_ST,mov_speed_TH)は 0-50 の範囲で与える。（0：最低速度、50:最大速度）
//スロットル、サーボモータの値(pos)の範囲は、 0≦ pos ≦180 で与える。
//myservo.write 関数には回転角を絶対的な位置で与える。例) 90°から 45°反時計回りに動いてほしいときは、-45ではなく、45を関数に入力する。

/*ステアとスロットルの位置を記憶する変数*/
int CH1 = center_pos; //CH1:ステア
int CH2 = neutral_pos; //CH2:スロットル

/*プログラムの流れを制御する変数*/
int flag = 0; //アプリの起動状態を管理する変数。アプリがバックグラウンドに入ったときは1に設定する。アプリがバックグラウンドから復帰し、アプリのPボタンが押されたら0に設定してラジコン操作を有効にする。
char input = 'C'; //入力信号
unsigned long t1 = 0; //データ受信時間
unsigned long t2 = 0; //割り込み時の時間
unsigned long td = 0; //データ受信時間と割り込み時間の差
unsigned long t_loss_ST = 0; //ステア操作で失われた時間
unsigned long t_loss_TH = 0; //スロットル操作で失われた時間
unsigned long curr ; /*現在時刻を取得*/
unsigned long prev_ST = 0; /*前時刻を保存*/
unsigned long prev_TH = 0; /*前時刻を保存*/

/*Bluetooth通信に必要*/
BluetoothSerial ESP_BT; //ESP_BTという名前でオブジェクトを定義

/*割り込み関数onTimerで実行される内容*/
void IRAM_ATTR onTimer() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); //前の出力と反転して点灯（チカチカする）
  t2 = millis();  //割り込み時の時間を測定（100ms間隔）
  td = t2 - t1; //割り込み時の時間と命令取得時間の差を取る。この差が大きいとき、アプリは起動していない状態。
}

/*ステアを操作する関数*/
void change_ST_pos(int goal_pos, unsigned long mov_speed) {
  unsigned long delay_time = 50 - mov_speed; //処理を遅くする時間（この値が大きいとゆっくりな操作に）

  if ((curr - prev_ST) >= delay_time) {
    /*ステアを切る処理*/
    //右折
    if (goal_pos - CH1 > 0) {
      CH1++;
      myservo.write(CH1);
      //左折
    } else if (goal_pos - CH1 < 0) {
      CH1--;
      myservo.write(CH1);

      //その他→ステアはそのまま
    } else {
      myservo.write(CH1);
    }
    prev_ST = curr; //前回に処理を実行した時刻を現在時刻に更新
  }

}

/*スロットルを操作する関数*/
void change_TH_pos(int goal_pos, unsigned long mov_speed) {
  unsigned long delay_time = 50 - mov_speed; //処理を遅くする時間（この値が大きいとゆっくりな操作に）

  if ((curr - prev_TH) >= delay_time) {
    /*スロットルを操作する処理*/
    //前進
    if (goal_pos - CH2 > 0) {
      CH2++;
      myesc.write(CH2);
      //後退
    } else if (goal_pos - CH2 < 0) {
      CH2--;
      myesc.write(CH2);

      //その他→スロットルはそのまま
    } else {
      myesc.write(CH2);
    }
    prev_TH = curr; //前回に処理を実行した時刻を現在時刻に更新
  }
}

void setup() {
  pinMode(LED_PIN_ACTIVE, OUTPUT); //LEDの点灯ピンを出力用に設定
  Serial.begin(115200); //シリアルモニタで確認用
  ESP_BT.begin("ESP32_RC_Receiver"); //接続画面で表示される名前を設定 ★好きな名前にしてよい
  myservo.attach(SERVO_PWM_PIN); //サーボモータのPWM端子とArduinoの4番ピンを接続 ★回路と対応した番号にする
  myesc.attach(ESC_PWM_PIN); //ESCのPWM端子とArduinoの16番ピンを接続 ★回路と対応した番号にする

  /*勝手には走りださないように設定*/
  myservo.write(center_pos); // ステアを中心(Center)に
  myesc.write(neutral_pos);  //中立(Neutral)

  /*割り込み処理（アプリ停止の検知に必要）の設定*/
  pinMode(LED_PIN, OUTPUT); //LEDの点灯ピンを出力用に設定
  timer = timerBegin(0, 80, true); //80クロック1カウント
  timerAttachInterrupt(timer, &onTimer, true); //onTimerという名前の関数で割り込み
  timerAlarmWrite(timer, 1000000 * 0.1, true); //80クロック×1000000カウント=1秒、1*0.1=100[ms]
  timerAlarmEnable(timer); //タイマー有効化
}

void loop() {
  curr = millis();
  /*available()で受信した信号があるか確認*/
  if (ESP_BT.available()) {
    t1 = millis(); //データを取得した時間を記録
    input = ESP_BT.read(); //受信したテキストを変数inputに保存
  }

  /*Pボタンが押されたらラジコンの操作を有効する（安全のため）*/
  if (flag == 1 && input == 'C') {
    flag = 0;
  }

  /*アプリ起動時（flag=0のとき）は操作を有効にする*/
  if (flag == 0) {

    /*命令に基づいてラジコンを制御*/
    if (input == 'A') {
      change_ST_pos(center_pos, mov_speed_ST); // ステアを中心(Center)に
      change_TH_pos(forward_max, mov_speed_TH); //前進(Forward)
    } else if (input == 'B') {
      change_ST_pos(center_pos, mov_speed_ST); // ステアを中心(Center)に
      change_TH_pos(backward_max, mov_speed_TH); //後退(Backward)
    } else if (input == 'C') {
      change_ST_pos(center_pos, mov_speed_ST); // ステアを中心(Center)に
      change_TH_pos(neutral_pos, mov_speed_brk); //中立(Neutral)
    } else if (input == 'D') {
      change_ST_pos(left_max, mov_speed_ST); // ステアを左(Left)に切る
      change_TH_pos(neutral_pos, mov_speed_brk); //中立(Neutral)
    } else if (input == 'E') {
      change_ST_pos(right_max, mov_speed_ST); // ステアを右(Right)に切る
      change_TH_pos(neutral_pos, mov_speed_brk); //中立(Neutral)
    } else if (input == 'F') {
      change_ST_pos(left_max, mov_speed_ST); // ステアを左(Left)に切る
      change_TH_pos(forward_max, mov_speed_TH); //前進(Forward)
    } else if (input == 'G') {
      change_ST_pos(right_max, mov_speed_ST); // ステアを右(Right)に切る
      change_TH_pos(forward_max, mov_speed_TH); //前進(Forward)
    } else if (input == 'H') {
      change_ST_pos(left_max, mov_speed_ST); // ステアを左(Left)に切る
      change_TH_pos(backward_max, mov_speed_TH); //後退(Backward)
    } else if (input == 'I') {
      change_ST_pos(right_max, mov_speed_ST); // ステアを右(Right)に切る
      change_TH_pos(backward_max, mov_speed_TH); //後退(Backward)
    } else {
      change_ST_pos(center_pos, 50); // ステアを中心(Center)に
      change_TH_pos(neutral_pos, 50); //中立(Neutral)
    }
  }


  /*シリアルモニタで確認用（ラジコンの制御には無関係）*/
  Serial.print("Received signal=");
  Serial.print(input);
  Serial.print(",");
  Serial.print("t1=");
  Serial.print(t1);
  Serial.print(",");
  Serial.print("t2=");
  Serial.print(t2);
  Serial.print(",");
  Serial.print("t2-t1=");
  Serial.print(td);
  Serial.print(",");

  /*アプリがバックグラウンドに入ったとき（もしくは閉じたとき）、車体を停止させる処理（安全のため）*/
  if (td > 150) {
    Serial.print("Transmitter is NOT active"); //NOT active：アプリはバックグラウンド状態（もしく閉じている）
    Serial.print("\n");
    myservo.write(center_pos); // ステアを中心(Center)に
    myesc.write(neutral_pos);  //中立(Neutral)
    input = ' ';
    digitalWrite(LED_PIN_ACTIVE, LOW); //アプリがバックグラウンドに入ったとき（もしくは閉じたとき）はLEDを消灯
    flag = 1; //アプリがバックグラウンド状態に入った（もしく閉じた）ときはflag=1に設定
  } else {
    Serial.print("Transmitter is active"); //active：アプリの画面が開いている
    Serial.print("\n");
    digitalWrite(LED_PIN_ACTIVE, HIGH); //アプリ起動時（画面を開いているとき）にはLEDを点灯
  }


}
