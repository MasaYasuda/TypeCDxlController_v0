/*============================================================================
Type-C Dxl Controller v0用プログラム

プログラム作成日：2025/08/05 (MasaYasuda)
最終更新日：2025/08/05 (MasaYasuda)

機能: 2つのボタンで回転方向、ノブで速度調整
対象: ATtiny1616
必須ライブラリ: SpenceKonde/megaTinyCore
※ 導入法：【Arduino】新しいATtiny(tinyAVR 0/1/2シリーズ)やmegaAVR 0シリーズなどで採用されたUPDIによる書き込み方法！ Arduino IDEで環境を構築する！ | ぶらり＠web走り書き(https://burariweb.info/electronic-work/arduino-updi-writing-method.html)
配線: ボタンピンは内部プルアップ
補足:
・ピン配置 参照：megaTinyCore/megaavr/extras/ATtiny_x16.gif at master · SpenceKonde/megaTinyCore(https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/ATtiny_x16.gif)
============================================================================*/

#include "Dxl.h"

// Dynamixelインスタンス生成（ID:1, Serial使用）
Dxl Dxl1(1, &Serial);

// ピン定義
const int16_t PIN_BUTTON_CW = 32;      // 正回転ボタン
const int16_t PIN_BUTTON_CCW = 1;      // 逆回転ボタン
const int16_t PIN_VOLUME_VELOCITY = 2; // 速度調整ボリューム（可変抵抗）
const int16_t MAX_VELOCITY = 512;

void setup()
{
  delay(500);                  // 少し待機
  Dxl1.factory_reset();        // Dynamixelを工場出荷状態にリセット,ボーレートは57600に設定,内部で自動Serial.begin(57600)
  Dxl1.servo_velocitylimit(MAX_VELOCITY);
  Dxl1.servo_control(1); // 制御モード設定（1:速度制御）
  delay(500);            // 少し待機
  Dxl1.servo_torque(1);  // トルクON
}

void loop()
{
  // ジョイスティックX軸の値を読み取り、目標速度を計算
  int16_t volumeVelocity = analogRead(PIN_VOLUME_VELOCITY); // ボリューム値取得
  char direction = 0;
  if !digitalRead (PIN_BUTTON_CW)
    direction += 1;
  if !digitalRead (PIN_BUTTON_CCW)
    direction += -1;
  int16_t velocity_abs = int16_t(map(volumeVelocity, 0, 1023, 0, MAX_VELOCITY)); // 0~MAX_VELOCITYにマッピング
  int16_t goalSpeed = velocity_abs * direction;
  // Dynamixelに目標速度を書き込み
  Dxl1.servo_speed(goalSpeed);
}
