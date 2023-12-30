# MD-DC01

DCブラシモータをセンサレス速度制御することが可能なモータドライバMD-DC01のためのファームウェアリポジトリです。

センサレス速度制御は出力無効タイミングでの逆起電力計測により実現します。

# ハードウェア

## 外観・接続

![MD-DC01 外観・接続図](doc/image/MD-DC01_connections.png)

## 回路図

[MD-DC01 V1.0](doc/MD-DC01_v1.0.pdf)

## 仕様

- 電源入力
  - 6 ～ 24V
- 出力
  - 電圧：入力と一致
    - ※逆起電力が計測できるのは16V程度まで
  - 電流：MAX 3A
- PCとの通信方法
  - UART
    - オンボードUSB-シリアル変換(CH340)搭載
    - UART上でPC側と絶縁
    - 115200 1N8
      - MAX 150kbps
- 汎用コネクタ
  - J2 エンコーダ
    - 5Vトレラント
    - 基板上の未実装抵抗に部品を実装することで、供給電圧・プルアップ電圧の設定が可能
  - J4 シリアル通信
    - UART or I2C
    - 基板上の未実装抵抗に部品を実装することで、プルアップ可能
  - J5 DAC x2
    - デバッグ用
  - J6,7 ADC x2
    - 目標値入力等に利用
  - J8 GPIO x2
    - 汎用

# 開発環境

- STM32CubeMX
  - 6.9.2 or Higher
- STM32Cube MCU Package for STM32F0 Series
  - 1.6.1

## 動作確認環境

- STM32CubeCLT
- VSCode
  - STM32 VS Code Extension

# ビルド手順

1. STM32CubeMX で ioc ファイルを開く
1. GENERATE CODE でコード生成する
1. Inc,Srcのxprintfをビルド対象に追加する
1. 各自環境でコンパイル
