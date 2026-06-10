#include <Arduino.h>
#include <RotaryEncoder.h>
#include <NeoPixelBusLg.h>
#include <BleGamepad.h> 

// -------------------------------------------------------------------- //
// LEDの設定
#define LED_COUNT 9 // LED数 SimHubの設定と合わせた方が良い
#define RIGHTTOLEFT 0 // LEDの向き 0:左から右 1:右から左
#define TEST_MODE 1 // テストモード 1:起動時にLEDを初期色で全部点灯させる 0:起動時はLED消灯
#define LUMINANCE_LIMIT 150 // LEDの明るさ制限 0～255
#define colorSpec NeoGrbFeature // LEDの色の並び
#define method NeoEsp32Rmt0Ws2812xMethod // ESP32のRMTを使う方法。
#define DATA_PIN D0 // データ出力できるピンは限られているっぽい
// Simhub接続前の色 赤
auto initialColor = RgbColor(120, 0, 0);

// LED 全部のRGB情報
uint8_t ledData[LED_COUNT*3]; // ledData[0~2] → [0]:LED#0のR [1]:LED#0のG [2]:LED#0のB ... 以降繰り返し
SemaphoreHandle_t ledMutex;  // 排他制御用

// LED制御用
NeoPixelBusLg<colorSpec, method, NeoGammaTableMethod> neoLedStrip(LED_COUNT, DATA_PIN);

// -------------------------------------------------------------------- //
// エンコーダーのピン定義
#define ENC_A  D7
#define ENC_B  D8
#define ENC2_A  D9
#define ENC2_B  D10

// エンコーダーのホールド管理用
const uint8_t HOLD_THRESHOLD = 5;
const uint16_t ENC_HOLD_TIME = 10; // エンコーダーボタンの維持時間(ms)
unsigned long enc1_timer = 0;
unsigned long enc2_timer = 0;
uint8_t enc1_hold_counter = 0;
uint8_t enc2_hold_counter = 0;

// エンコーダーのインスタンス
RotaryEncoder *encoder = nullptr;
RotaryEncoder *encoder2 = nullptr;

// -------------------------------------------------------------------- //
// キーマトリックスの設定

#define ROW_COUNT 3
#define COL_COUNT 3
#define ON LOW
#define OF HIGH

const byte pins_row[ROW_COUNT] = {D1, D2, D3};
const byte pins_col[COL_COUNT] = {D4, D5, D6};
const uint8_t DEBOUNCE_COUNT = 3; // 何回のスキャンで確定させるか

// ボタン番号のマッピング
const byte keyMap[ROW_COUNT][COL_COUNT] = {
  {BUTTON_7, BUTTON_8, BUTTON_9},
  {BUTTON_4, BUTTON_2, BUTTON_6},
  {BUTTON_3 ,BUTTON_1, BUTTON_5}
};

// 状態管理用
bool swState[ROW_COUNT][COL_COUNT] = {
  {false, false, false},
  {false, false, false},
  {false, false, false}
};
// デバウンス用カウンタ 
uint8_t swCounters[ROW_COUNT][COL_COUNT] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

// -------------------------------------------------------------------- //

// BLEゲームパッドのインスタンス
BleGamepad bleGamepad("BLE_Wheel");

// LEDタスクハンドル
TaskHandle_t ledTaskHandle = NULL;

// 入力管理タスクハンドラ
TaskHandle_t inputTaskHandle = NULL;


// エンコーダーの状態を更新
void checkPosition()
{
  encoder->tick();
  encoder2->tick();
}

// -------------------------------------------------------------------- //
// Reportのレイアウトについて → https://manual.simhubdash.com/device-definition-authoring/device-communication-protocols#simhub-standard-hid-protocol#simhub-standard-hid-protocol
// buffer[0] : LEDの更新開始位置
// buffer[1] : LEDの更新数
// buffer[2] : 更新フラグ 0:まだ続きがある 1:これで最後 2:接続通知（オプション機能）3:切断通知（オプション機能）
// buffer[3]～ : LEDの色情報 R G B が1バイトずつ繰り返し
// 例）LED_COUNT=20のとき、全部更新なら buffer[3]～buffer[62] にLED0～LED19のRGBが入る
// 本来はbuffer[0]にReportIDが入るはずだが、BLEGamepadが（？）返してこないので、上記で実装
// 64バイト設定の場合、(64-4)/3=20 でLED20個分が最適か？(20以上にしても分割されるだけのはず）
// -------------------------------------------------------------------- //
// LED更新タスク
void ledTask(void *pvParameters) {  
  while (1) {
    // 通知受け取り
    uint32_t thread_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // 通知があったらLEDを更新する
    if (thread_notification > 0) {
      uint8_t r;
      uint8_t g;
      uint8_t b;
      for(uint8_t j = 0; j < LED_COUNT; j++){
        int dataIndex = (j*3);
        r = ledData[dataIndex];
        g = ledData[dataIndex+1];
        b = ledData[dataIndex+2];
        if (RIGHTTOLEFT == 1) {
          neoLedStrip.SetPixelColor(LED_COUNT - j - 1, RgbColor(r, g, b));
        } else {
          neoLedStrip.SetPixelColor(j, RgbColor(r, g, b));
        }
      }
      // 時間かかるけどマルチタスクなので
      neoLedStrip.Show();
    }
  }
}

uint16_t outputBufferTime =0;
// Simhubからのデータ受信タスク
void hidOutputListenerTask(void *pvParameters) {
  while (1) {
    if (bleGamepad.isConnected() && bleGamepad.isOutputReceived()) {
      
      // データを受信したら共有バッファに書き込む
      uint8_t* buffer = bleGamepad.getOutputBuffer();
      uint8_t start = buffer[0];
      uint8_t count = buffer[1];
      uint8_t flag = buffer[2];

      // 共有バッファへの書き込み
      // ロックをしてから書き込み開始
      if (xSemaphoreTake(ledMutex, portMAX_DELAY)) {
        int index = start * 3;
        int dataCount = count * 3;
        if (index + dataCount <= LED_COUNT * 3) {
          memcpy(&ledData[index], &buffer[3], dataCount);
        }
        //解放
        xSemaphoreGive(ledMutex);
      }
      // データフレームが終端なら通知を送る（更新タスクを実行する）
      if (flag == 1 && ledTaskHandle != NULL) {
        xTaskNotifyGive(ledTaskHandle);
      }
    }
    // 1msごとに繰り返し
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}


// 入力情報制御用
void inputTask(void *pvParameters) {
  // 30msごとに繰り返し処理
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); 

  while (1) {
    if (bleGamepad.isConnected()) {
      // キーマトリックスのスキャン 
      for (byte i = 0; i < ROW_COUNT; i++) {
        digitalWrite(pins_row[i], LOW);
        delayMicroseconds(10); // 信号安定待ち
        for (byte j = 0; j < COL_COUNT; j++) {
          bool rawReading = (digitalRead(pins_col[j]) == ON);
          if (rawReading != swState[i][j]) {
            swCounters[i][j]++; 
            if (swCounters[i][j] >= DEBOUNCE_COUNT) {
              swState[i][j] = rawReading;
              swCounters[i][j] = 0;
              if (swState[i][j]) {
                bleGamepad.press(keyMap[i][j]);
              } else {
                bleGamepad.release(keyMap[i][j]);
              }
          }
          } else {
            swCounters[i][j] = 0;
          }
        }
        digitalWrite(pins_row[i], HIGH);
      }

      // エンコーダーの状態チェック ほぼライブラリにおまかせ
      // エンコーダー1
      int newPos = encoder->getPosition();
      static int pos = 0;
      if (pos != newPos) {
        int dir = (int)(encoder->getDirection());
        if (dir == 1)       { bleGamepad.press(BUTTON_10); } 
        else if (dir == -1) { bleGamepad.press(BUTTON_11); }
        enc1_hold_counter = HOLD_THRESHOLD;
        pos = newPos;
      } else if (enc1_hold_counter > 0) {
        enc1_hold_counter--;
        if (enc1_hold_counter == 0) {
          bleGamepad.release(BUTTON_10);
          bleGamepad.release(BUTTON_11);
        }
      }

      // エンコーダー2
      int newPos2 = encoder2->getPosition();
      static int pos2 = 0;
      if (pos2 != newPos2) {
        int dir2 = (int)(encoder2->getDirection());
        if (dir2 == 1)       { bleGamepad.press(BUTTON_12); } 
        else if (dir2 == -1) { bleGamepad.press(BUTTON_13); }
        enc2_hold_counter = HOLD_THRESHOLD;
        pos2 = newPos2;
      } else if (enc2_hold_counter > 0) {
        enc2_hold_counter--;
        if (enc2_hold_counter == 0) {
          bleGamepad.release(BUTTON_12);
          bleGamepad.release(BUTTON_13);
        }
      }
      bleGamepad.sendReport();
    }
    // 周期がくるまで待機（30ms - ここの処理時間）
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  // キーマトリックス用ピンの初期化
  for (byte i = 0; i < ROW_COUNT; i++) {
    pinMode(pins_row[i], OUTPUT);
    digitalWrite(pins_row[i], HIGH);
  }
  for (byte i = 0; i < COL_COUNT; i++) {
    pinMode(pins_col[i], INPUT_PULLUP);
  }

  // エンコーダーのインスタンス （実体）
  encoder = new RotaryEncoder(ENC_A, ENC_B, RotaryEncoder::LatchMode::FOUR3);
  encoder2 = new RotaryEncoder(ENC2_A, ENC2_B, RotaryEncoder::LatchMode::FOUR3);

  // エンコーダーの割り込み設定
  attachInterrupt(digitalPinToInterrupt(ENC_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), checkPosition, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC2_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), checkPosition, CHANGE);

  // BLEの設定
  BleGamepadConfiguration config;
  config.setAutoReport(false); // 一括更新の方がいいかも？
  config.setControllerType(CONTROLLER_TYPE_GAMEPAD); 
  config.setButtonCount(13);
  config.setHatSwitchCount(0);
  config.setWhichAxes  (false,false,false,false,false,false,false,false); 
  config.setEnableOutputReport(true); // PC側からのデータを受け取る機能を有効化
  config.setOutputReportLength(63); //一度に受け取るデータ長 SimHubの設定を合わせる必要あり 
                                    // ReportIDを含めない長さ？のようなのでSimHub設定値-1 を指定（実際は64バイト）
  config.setHidReportId(0x05); // ReportID SimHubの設定を合わせる
  // config.setVid(0xe502); // ベンダーID 初期値：0xe502
  // config.setPid(0xbbab); // プロダクトID 初期値：0xbbab

  bleGamepad.begin(&config);

  neoLedStrip.Begin();  // LEDの処理開始
  neoLedStrip.Show();

  // LEDの初期化 全部赤にする
  if (TEST_MODE)
  {
      for (int i = 0; i < LED_COUNT; i++)
      {
          neoLedStrip.SetPixelColor(i, initialColor);
      }
      neoLedStrip.Show();
  }
  neoLedStrip.SetLuminance(LUMINANCE_LIMIT);

  ledMutex = xSemaphoreCreateMutex();

  // LED更新タスク
  xTaskCreate(ledTask,"LED_TASK",2048,NULL,2,&ledTaskHandle);

  // 受信監視タスク（優先度3：表示タスクより高くする）
  xTaskCreate(hidOutputListenerTask, "HID_TASK", 2048, NULL, 3, NULL);

  // 入力更新用タスク
  xTaskCreate(inputTask, "INPUT_TASK", 4096, NULL, 3, &inputTaskHandle);

}

void loop() {
}