/* Happy_RC_Receiver *****************************
  Download a Transmitter App:https://github.com/TomokiIkegami/Happy_RC_Driver/raw/main/Happy_RC_Driver.apk
  About this Project:https://github.com/TomokiIkegami/Happy_RC_Driver

  ◆ 動作
  1.スマホアプリ(Happy_RC_Receiver)から命令（ステア、スロットルのパルス幅）を取得
  2.取得した命令に基づいて、RCを前後左右に操作
  3.動作1,2と同時に、車速を計算してスマホに送信
  ◆ 機能
  ・アプリを開いていないときは、車体を停止させる安全機能を装備（Dragブレーキ付きESCを使用すること）
  ・アプリ再起動時には、スロットルを中立に戻さないと走行しない（アプリ再起動時に急に走行する危険を防ぐため）
  ・カクカクではなく滑らかな走行が可能
  ◆ 補足
  ・プログラム中で★マークがついている部分は、自分の装置に合わせて調整
*************************************************************/

/*ハードウェアの接続ピンの設定*/
#define LED_PIN 2 //走行には無関係。2番ピンにLEDのアノード(+)を接続すると割り込み処理の間隔(100ms)でLEDが点滅
#define LED_PIN_ACTIVE 17 //走行には無関係。17番ピンにLEDのアノード(+)を接続すると、アプリ起動時のみLEDが点灯
#define SERVO_PWM_PIN 4  //サーボモータのPWMピン（信号入力ピン）をESP32の4番ピンに接続 ★回路と対応した番号にする
#define ESC_PWM_PIN 16   //ESCのPWMピン（信号入力ピン）をESP32の16番ピンに接続 ★回路と対応した番号にする

/*タイマーの定義*/
hw_timer_t*timer = NULL;

/*マルチタスクに使うタスクハンドルを定義*/
TaskHandle_t thp[1]; // マルチスレッドのタスクハンドル格納用の変数（スレッドの数に応じて添え字の数を変更）
int num = 0;

/*ライブラリ*/
#include "BluetoothSerial.h"  //ESP32のBluetooth通信に使用
#define PI 3.141592653589793 //円周率

/*Bluetooth通信に必要*/
BluetoothSerial ESP_BT;  //ESP_BTという名前でオブジェクトを定義

/*ステアとスロットルの設定*/
int PWM_period = 20000;  //PWM周期をT=20000[μs]に設定
int center_pos = 1500;   //ステア中心位置 [サーボモータ中心位置のパルス幅 (1500)]
int neutral_pos = 1500;  //中立位置 [スロットル中立位置のパルス幅 (1500)]
int CH1 = center_pos;    //パルス幅Tonを1500[μs]（中立）に設定。データシートにはLeft=900[μs]、Neutral=1500[μs]、Right=2100[μs]と記載されていた。(HiTEC DB777WP)
int CH2 = neutral_pos;   //パルス幅Tonを1500[μs]（中立）に設定。

/*プログラムの流れを制御する変数*/
int flag = 0;  //アプリの起動状態を管理する変数。アプリがバックグラウンドに入ったときは1に設定する。アプリがバックグラウンドから復帰し、アプリのPボタンが押されたら0に設定してラジコン操作を有効にする。
String input = "1500,1500";   //入力信号
unsigned long t1 = 0;         //データ受信時間
unsigned long t2 = 0;         //割り込み時の時間
unsigned long td = 0;         //データ受信時間と割り込み時間の差

/*パラメータ設定*/
const int sense_pin = 25;  //★回路に合わせる。今回は25番ピンを使用。
const double z1=14; //ベベルギアの歯数[枚]
const double z2=38; //リングギアの歯数[枚]
const double D=95; //タイヤの直径[mm]

/*センサ関連の設定*/
const int th = 3800;  //センサ閾値
int val = 0;          //センサ出力値
int output = 0;       //パルス出力（0/1）
int y[50];            //パルス列を保存する配列
int count = 0;        //パルスのカウント値

/*速度計算関連の変数*/
double f = 0;             //周波数[Hz]
double v,v_kmh = 0;       //速度v:[m/s], 速度v_kmh:[km/h]
int f_int = 0 ;           //周波数（整数）
int v_kmh_int = 0 ;       //周波数（整数）
uint8_t f_uint8 = 0;      //周波数（符号なし8bit[2byte]整数）
uint8_t v_kmh_uint8 = 0;  //周波数（符号なし8bit[2byte]整数）

/*割り込み関数onTimerで実行される内容*/
void IRAM_ATTR onTimer() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  //前の出力と反転して点灯（チカチカする）
  t2 = millis();                                 //割り込み時の時間を測定（100ms間隔）
  td = t2 - t1;                                  //割り込み時の時間と命令取得時間の差を取る。この差が大きいとき、アプリは起動していない状態。
}

/*PWM制御でサーボモータの角度を制御する関数*/
void change_ST_pos(int goal_pos) {
  /*20000[μs]周期で幅CH1[μs]のパルス波生成*/
  digitalWrite(SERVO_PWM_PIN, HIGH);  //PWMピンの電圧を5[V]に
  delayMicroseconds(goal_pos);        //パルス幅(goal_pos=900~2100[μs]：DB777WPの場合)
  digitalWrite(SERVO_PWM_PIN, LOW);   //PWMピンの電圧を0[V]に
  delayMicroseconds(PWM_period);      //制御周期（T=20000[μs]）
}

/*PWM制御でスロットルを制御する関数*/
void change_TH_pos(int goal_pos) {
  /*20000[μs]周期で幅CH1[μs]のパルス波生成*/
  digitalWrite(ESC_PWM_PIN, HIGH);  //PWMピンの電圧を5[V]に
  delayMicroseconds(goal_pos);      //パルス幅(goal_pos=900~2100[μs]：DB777WPの場合)
  digitalWrite(ESC_PWM_PIN, LOW);   //PWMピンの電圧を0[V]に
  delayMicroseconds(PWM_period);    //制御周期（T=20000[μs]）
}

/*文字列を分割するための関数（分割数を戻り値とする）*/
void split(String data, char delimiter, String *dst) {
  // data:分割したい文字列、delimiter:区切り文字、*dst:分割後の文字を入れる配列
  int index = 0;                   //分割後の文字をどこに入れるか決める
  int i;                           //カウンタ変数
  int datalength = data.length();  //文字列の長さを取得

  for (i = 0; i < datalength; i++) {  //文字列を1文字ずつスキャン
    char tmp = data.charAt(i);        //tmp:文字を入れておく一時的な変数
    if (tmp == delimiter) {
      index++;  //区切り文字があったら、配列の添え字を増やす
    } else {
      dst[index] += tmp;  //文字を1文字ずつ結合して、文字列を作成
    }
  }
}
void Task2(void *args){
  int i;                //カウンタ変数
  while(1){ // 無限ループ作成
    /*フォトリフレクタから値を読み取り、パルスの生成*/
    for (i = 0; i < 50; i++) { //0.01×50=0.5[s] 間隔で周波数を計算
      val = analogRead(sense_pin);  //センサの値などを読む

      /**パルス作成**/
      if (val < th) { //センサの前にシャフトの目印が通過したらパルス値を1にする
        output = 1;
      } else {
        output = 0;
      }
      y[i] = output;  //パルス出力値を保存
      delay(10); //0.01[s] ごとに測定
    }

    /**パルス数計算*/
    for (i = 0; i < 49; i++){ //0.01×50=0.5[s] 間隔で周波数を計算
      if((y[i]^y[i+1])==1){
        count++; //排他的論理和（EX-OR）でパルスの立ち上がり/立ち下がりを計算
      }    
    }

    /*周波数の計算、スマホに値を送信*/
    f=((double)count/2.0)*(1.0/0.5); //周波数f[Hz]を計算
    v=(D*0.001/2.0)*2*PI*f*(z1/z2); //周波数から速度[m/s]を計算
    v_kmh=v*3600/1000; //[m/s]から[km/h]に換算
    v_kmh_int = (int)(v_kmh*10); //速度を整数値に変換（10倍してスケール速度に変換）
    v_kmh_uint8 =  v_kmh_int; //速度を符号なし8bit整数に変換
    ESP_BT.write(v_kmh_uint8); //速度をスマホに送信
    count=0; //カウント値をリセット

    /* シャフト周波数や車速をシリアルモニタに表示 */
    // f_int = (int)f; //周波数を整数に変換
    // f_uint8 = f_int; //周波数を符号なし8bit整数に変換
    // Serial.print("f = "); Serial.print(f); Serial.print(" [Hz], "); //シリアルモニタに周波数（シャフト回転数）と速度を表示
    // Serial.print("v_kmh = "); Serial.print(v_kmh); Serial.print(" [km/h]\n"); //シリアルモニタに周波数（シャフト回転数）と速度を表示
    // ESP_BT.write(f_uint8); //周波数をスマホに送信ESP_BT.write(f_uint8); //周波数をスマホに送信

  }
}

void setup() {
  /*出力ピン設定*/
  pinMode(SERVO_PWM_PIN, OUTPUT);     //サーボモータのピンへ、ArduinoからPWM出力するように設定
  pinMode(ESC_PWM_PIN, OUTPUT);       //ESCのピンへ、ArduinoからPWM出力するように設定
  Serial.begin(115200);               //シリアルモニタで確認用。伝送速度を115200[bps]に設定
  ESP_BT.begin("ESP32_RC_Receiver");  //接続画面で表示される名前を設定 ★好きな名前にしてよい

  /*割り込み処理（アプリ停止の検知に必要）の設定*/
  pinMode(LED_PIN, OUTPUT);                     //LEDの点灯ピンを出力用に設定
  timer = timerBegin(1000000);              //周波数を1MHzに指定（1µs毎にカウントアップする）　※ ESP32 Core 2.x系 → 3.x系に伴い変更した
  timerAttachInterrupt(timer, &onTimer);  //onTimerという名前の関数で割り込み
  timerAlarm(timer, 100000, true, 0);  // (1/周波数)*カウント数=(1/1000000)*100000 = 0.1 [s]ごとに割り込みを発生 ※ ESP32 Core 2.x系 → 3.x系に伴い変更した

  /*マルチタスクの設定*/
  xTaskCreatePinnedToCore(Task2,"Task2",4096,NULL,1,&thp[0],0); //(タスク名,"タスク名",スタックメモリサイズ,優先度(1~24),タスクハンドルのポインタ,コアID(0か1))
}

void loop() {

  if (ESP_BT.available()) {               //available()で受信した文字があるか確認。あればif文内の処理を実行。
    t1 = millis();                        //データを取得した時間を記録
    input = ESP_BT.readStringUntil(';');  //文字をセミコロン(;)まで読んで、文字列として変数inputに保存。
    String cmds[2] = { "\0" };            //この配列に分割された信号を格納

    split(input, ',', cmds);  //文字列inputを分割＆cmdsに保存

    CH1 = (cmds[0]).toInt();  //CH1（ステア）の値
    CH2 = (cmds[1]).toInt();  //CH2（スロットル）の値

    // /*受信したパルス幅の表示*/
    // Serial.print(CH1);
    // Serial.print(',');
    // Serial.print(CH2);
    // Serial.print('\n');
  }

  /*Pボタンが押されたらラジコンの操作を有効する（安全のため）*/
  if (flag == 1 && CH2 == 1500) {
    flag = 0;
  }
  /*アプリ起動時（flag==0のとき）は操作を有効にする*/
  if (flag == 0) {
    change_ST_pos(CH1);  //受信したパルス幅でサーボモータの角度を制御
    change_TH_pos(CH2);  //受信したパルス幅でスロットルを制御
  }

  /*シリアルモニタで確認用（ラジコンの制御には無関係）*/
  Serial.print("v_kmh=");
  Serial.print(v_kmh);
  Serial.print(",");
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
    Serial.print("Transmitter is NOT active");  //NOT active：アプリはバックグラウンド状態（もしく閉じている）
    Serial.print("\n");
    change_TH_pos(neutral_pos);  //中立(Neutral)
    input = " ";
    digitalWrite(LED_PIN_ACTIVE, LOW);  //アプリがバックグラウンドに入ったとき（もしくは閉じたとき）はLEDを消灯
    flag = 1;                           //アプリがバックグラウンド状態に入った（もしく閉じた）ときはflag=1に設定
  } else {
    Serial.print("Transmitter is active");  //active：アプリの画面が開いている
    Serial.print("\n");
    digitalWrite(LED_PIN_ACTIVE, HIGH);  //アプリ起動時（画面を開いているとき）にはLEDを点灯
  }

}
