#include <SPI.h>
#include <SD.h>
//2つともSDに書き込むために必要

#include <Wire.h>
//I2C

//指定したバスのポートを解放する関数
void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// ADXL345のデバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x53;
//センサ数に対応
int j;

// XYZレジスタ用のテーブル(6byte)
uint8_t RegTbl[6];

const int chipSelect = 4; //SDカード使用の場合
const int pinWriteSW = 2; //トグルスイッチ
int stateWriteSW; //SD writeの関数をONにするかのトリガースイッチ
unsigned long time; //プログラムの時間変数


void setup() {
  Serial.begin(115200);

  //ここを出力にしておかないとSPI端子がSlaveになりライブラリがうまく動かない。
  pinMode(SS, OUTPUT);
  pinMode(pinWriteSW, INPUT);
  stateWriteSW = 1; //スイッチと同時に起動させるなら1

  SD.begin(chipSelect);

  // マスタとしてI2Cバスに接続する
  Wire.begin();

  //以下ADXL345の初期設定
  // DATA_FORMAT(データ形式の制御)
  for (j = 0; j < 3; j++) {
    TCA9548A(j);
    Wire.beginTransmission(DEVICE_ADDRESS);
    // DATA_FORMATのアドレス
    Wire.write(0x31);
    // 「最大分解能モード」 及び 「±16g」 (0x0B == 1011)
    Wire.write(0x0B);
    // 「10bit固定分解能モード」 及び　「±16g」にする場合 (0x03 == 0011)
    // Wire.write(0x03);
    Wire.endTransmission();

    // POWER_TCL(節電機能の制御)
    Wire.beginTransmission(DEVICE_ADDRESS);
    // POWER_CTLのアドレス
    Wire.write(0x2d);
    // 測定モードにする
    Wire.write(0x08);
    Wire.endTransmission();
  }
  //  Wire.endTransmission();
}

void loop() {
  //SDカードにデータを着込む関数
  writingMode();
}
// タイミング調整用

void writingMode() {
  if (stateWriteSW == HIGH) {
    time = millis();

    for (j = 0; j < 3; j++) {
      TCA9548A(j);
      //      Serial.print("Now Connection...to");
      //      Serial.println(j);

      Wire.beginTransmission(DEVICE_ADDRESS);
      // XYZの先頭アドレスに移動する
      Wire.write(0x32);
      Wire.endTransmission();
      Wire.requestFrom(0x53, 6);

      // 6byteのデータを取得する
      int i;
      for (i = 0; i < 6; i++) {
        while (Wire.available() == 0 ) {}
        RegTbl[i] = Wire.read();
      }

      // データを各XYZの値に変換する(LSB単位)
      //8bit左シフトからのBit OR/ int16_tに変換
      int16_t x = (((int16_t)RegTbl[1]) << 8) | RegTbl[0];
      int16_t y = (((int16_t)RegTbl[3]) << 8) | RegTbl[2];
      int16_t z = (((int16_t)RegTbl[5]) << 8) | RegTbl[4];


      //data書き込みパート
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        //   Timeスタンプ
        if (j < 2) {
          dataFile.print("Time:");
          dataFile.print(time);
          dataFile.print("  ");

          // 各XYZ軸の加速度(m/s^2)を出力する
          //16gなので32の範囲を13bitで割ることでSI単位系に直している.

          dataFile.print("Xa");
          dataFile.print(j);
          dataFile.print(": ");
          dataFile.print(x);
          dataFile.print("  ");

          dataFile.print("Ya");
          dataFile.print(j);
          dataFile.print(": ");
          dataFile.print(y);
          dataFile.print("  ");

          dataFile.print("Za");
          dataFile.print(j);
          dataFile.print(": ");
          dataFile.print(z);
          dataFile.print("  ");

          dataFile.close();
        }
        else {

          //   Timeスタンプ

          dataFile.print("Time:");
          dataFile.print(time);
          dataFile.print("  ");
          // 各XYZ軸の加速度(m/s^2)を出力する
          //16gなので32の範囲を13bitで割ることでSI単位系に直している.

          dataFile.print("Xa");
          dataFile.print(j);
          dataFile.print(": ");
          dataFile.print(x);
          dataFile.print("  ");

          dataFile.print("Ya");
          dataFile.print(j);
          dataFile.print(": ");
          dataFile.print(y);
          dataFile.print("  ");

          dataFile.print("Za");
          dataFile.print(j);
          dataFile.print(": ");
          dataFile.println(z);

          dataFile.close();

        }
        //        Serial.print("X");
        //        Serial.print(j);
        //        Serial.print(": ");
        //        Serial.print( x * 0.0392266 );
        //        Serial.print("Y");
        //        Serial.print(j);
        //        Serial.print(": ");
        //        Serial.print( y * 0.0392266 );
        //        Serial.print("Z");
        //        Serial.print(j);
        //        Serial.print(": ");
        //        Serial.print( z * 0.0392266 );
        //        Serial.println(" m/s^2");
      }
      //  delay(10);
    }
  }
}
