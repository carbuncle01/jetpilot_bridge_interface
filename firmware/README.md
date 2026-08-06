# JPBB-01 Step 8 firmware

`jpbb_step8_final.hex`は、`jetpilot_bridge_interface`とJPB1で通信する
STM32G0B1CBT6向けStep 8ファームウェアです。

## この成果物の仕様

- JPB1コマンド受信: 100 Hz想定
- JPB1ステータス送信: 20 Hz
- 指令timeout: 200 ms
- heartbeat timeout: 500 ms
- PWMニュートラル: 1500 µs
- 前進スロットル: 1500 µsから1000 µs側
- ブレーキ／後進: 1500 µsから2000 µs側
- CH3 Low: RC/PROPO要求
- CH3 High: Jetsonホスト経路の許可

ブレーキと後進は同じ2000 µs側です。ESCによってはブレーキを保持すると
後進へ遷移するため、最初の確認は必ずタイヤを浮かせて行ってください。

## ビルド元

- firmware repository commit: `f36fd306add45793dc73c1707112047ff8755eaa`
- target: `step8_final`
- toolchain: Arm GNU Toolchain 13.2.1
- STM32CubeG0: v1.6.3
- local vehicle polarity adjustment:
  - `throttle=1.0` -> 1000 µs
  - `reverse=1.0` -> 2000 µs
  - `brake=1.0` -> 2000 µs

ビルド時のメモリ使用量はFLASH 31,380 bytes、RAM 6,688 bytesです。

## 整合性確認

```bash
cd firmware
sha256sum -c SHA256SUMS
```

## 書き込み

ST-LINKとSTM32CubeProgrammerを使い、`jpbb_step8_final.hex`を書き込んで
verify後にresetします。CLIを使う場合の例です。

```bash
STM32_Programmer_CLI -c port=SWD \
  -w jpbb_step8_final.hex -v -rst
```

実機書き込み後は、モーター電源を安全に遮断できる状態でUSB列挙、1500 µs
ニュートラル、CH3切替、指令timeout、USB切断、前進／ブレーキ／後進の順に
確認してください。
