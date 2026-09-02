# jetpilot_bridge_interface

`jetpilot_bridge_interface`は、JPBB-01 `jetpilot_bridge_board`をJetPilotの車両制御へ接続するROS 2パッケージです。

次の機能を一つのvehicle interfaceとして提供します。

- `/vehicle/control_cmd`からUSB CDCコマンドへの変換
- JPBB-01のRC入力、PWM出力、VBEC、動作経路、fault状態の受信
- CH3の2位置スイッチ（1000 µs付近=PROPO、2000 µs付近=ホスト許可）との連携
- USB切断、指令timeout、status timeoutの診断
- USB再接続後の明示的な安全再アーム
- `base_link`からカメラフレームへの固定TF

## 起動

```bash
ros2 launch jetpilot_bridge_interface jetpilot_bridge_interface.launch.xml \
  publish_description:=true
```

実車では`device`を安定した`/dev/serial/by-id/...`へ変更してください。

```yaml
/**:
  ros__parameters:
    device: /dev/serial/by-id/usb-JetPilot_JPBB-01_...
```

## ROSインターフェース

### Subscribe

| Topic | Type | 用途 |
| --- | --- | --- |
| `/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | 選択済み車両指令。launchで`/vehicle/control_cmd`へremap |
| `/operation_mode/state` | `jetpilot_msgs/msg/OperationModeState` | STOP／PROPO／Joy MANUAL／自律AUTOの状態 |
| `/steer_offset_inc` | `std_msgs/msg/Bool` | `true`でホスト経路のsteering offsetを`offset_step`増やす |
| `/steer_offset_dec` | `std_msgs/msg/Bool` | `true`でホスト経路のsteering offsetを`offset_step`減らす |

### Publish

| Topic | Type | 用途 |
| --- | --- | --- |
| `/operation_mode/request` | `jetpilot_msgs/msg/OperationModeRequest` | CH3のPROPO要求、基板fault、USB／指令断を反映 |
| `~/rc_channels` | `std_msgs/msg/Int32MultiArray` | CH1、CH2、CH3入力パルス幅 [µs] |
| `~/output_channels` | `std_msgs/msg/Int32MultiArray` | サーボ、ESC出力パルス幅 [µs] |
| `~/vbec_voltage` | `std_msgs/msg/Float32` | VBEC電圧 [V] |
| `~/active_path` | `std_msgs/msg/UInt8` | 0=DISABLED、1=MANUAL、2=AUTO、3=FAILSAFE |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 接続、timeout、基板fault状態 |

## USBプロトコル JPB1

ランタイム通信は改行区切りASCIIフレームとCRC-16/CCITT-FALSEを使用します。CRCは末尾CRCフィールドを除く文字列全体を対象とします。

### JetsonからSTM32

```text
JPB1,C,sequence,steering_milli,throttle_milli,reverse_milli,brake_milli,flags,CRC16
```

| Field | Range |
| --- | --- |
| `steering_milli` | -1000〜1000 |
| `throttle_milli` | 0〜1000 |
| `reverse_milli` | 0〜1000 |
| `brake_milli` | 0〜1000 |
| `flags bit 0` | 指令がtimeout内で有効 |
| `flags bit 1` | ホスト経路要求。Joy MANUALと自律AUTOのどちらでも使用 |

USBでは正規化指令を送り、STM32が実機校正値を使って1000〜2000 µsのPWMへ変換します。これにより安全PWM、出力制限、watchdogを基板側で完結できます。

ROS内部のsteering規約は正が左です。`steering_scale`は車両境界で指令へ乗算され、
JPBB-01の負が左という実機規約に合わせる既定値は`-1.0`です。
`steering_offset`はscale適用後のsteering指令へ加算され、結果は`[-1.0, 1.0]`へ
clampされます。既定のJoy設定では十字キー右／左がそれぞれ
`/steer_offset_inc`／`/steer_offset_dec`をpublishするため、PCA9685と同じ操作で
JPBBのホスト経路を調整できます。この値はJoy MANUALと自律AUTOに適用され、
基板内で直接選択されるPROPO経路には適用されません。起動時の初期値を残すには
parameter YAMLの`steering_offset`を更新してください。

### STM32からJetson

```text
JPB1,S,sequence,rx1_us,rx2_us,rx3_us,servo_us,esc_us,vbec_mv,selector,active_path,fault_bits,CRC16
```

| Field | Definition |
| --- | --- |
| `selector` | 0=PROPO、1=ホスト許可 |
| `active_path` | 0=DISABLED、1=RC、2=HOST、3=FAILSAFE |
| `fault_bits` | ファームウェア仕様で定義するbit mask。0はfaultなし |

`fault_bits`はbit 0から順にVBEC、USB、受信機喪失、CH3中間／不明、RC信号、
指令timeout、heartbeat timeout、指令値異常を表します。

## 制御権と安全境界

CH3がLow側なら受信機CH1／CH2を使用します。CH3がHigh側ならJetsonからの制御を許可しますが、Joy手動と自律走行の選択は従来どおりoperation modeとcommand muxが担当します。CH3 Highだけで自律AUTOへ切り替わることはありません。

STM32はCH3、RC入力、USB、コマンドCRCと範囲、VBEC、watchdogを独立して確認します。条件が崩れた場合は1500/1500 µsのSAFE出力へ遷移してホストをディスアームします。

### USBを抜いて差し直した場合

1. STM32はUSB断または指令200 ms超過を検出し、SAFE出力にしてホストをディスアームします。
2. bridge nodeはstatus timeout／I/Oエラーを検出し、operation modeへSTOPを要求します。
3. nodeは既定で1秒ごとに同じデバイスを再オープンします。
4. 再接続後はニュートラルの非アームフレームだけを送り、以前の操作量を再開しません。
5. CH3がホスト許可側でstatusが正常になった後、STOPからMANUALまたはAUTOを選び直すと、ニュートラル・ハンドシェイク後に再アームします。

Linux側では実車の`device`を`/dev/serial/by-id/...`に設定してください。

## JetPilot全体からの起動

```bash
./scripts/bringup.sh drive --vehicle jpbb
```

Joy手動はMANUAL、自律走行はAUTOを選択します。`command_mux_node`のロジックは変更していません。

## Step 8ファームウェア

実車用に確認中のStep 8 HEXを[`firmware`](firmware/README.md)へ同梱しています。
パッケージのビルド後はshareディレクトリにもinstallされます。

```bash
FIRMWARE_DIR="$(ros2 pkg prefix --share jetpilot_bridge_interface)/firmware"
sha256sum -c "${FIRMWARE_DIR}/SHA256SUMS"
```

このHEXはJPB1指令について、1500 µsをニュートラル、1000 µs側を
前進スロットル、2000 µs側をブレーキ／後進として変換します。
