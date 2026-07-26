# jetpilot_bridge_interface

`jetpilot_bridge_interface`は、JPBB-01 `jetpilot_bridge_board`をJetPilotの車両制御へ接続するROS 2パッケージです。

次の機能を一つのvehicle interfaceとして提供します。

- `/vehicle/control_cmd`からUSB CDCコマンドへの変換
- JPBB-01のRC入力、PWM出力、VBEC、動作経路、fault状態の受信
- CH3の2位置スイッチ（1000 µs=PROPO、2000 µs=AUTO）とoperation modeの同期
- USB切断、指令timeout、status timeoutの診断
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
| `/operation_mode/state` | `jetpilot_msgs/msg/OperationModeState` | AUTO許可要求の生成 |

### Publish

| Topic | Type | 用途 |
| --- | --- | --- |
| `/operation_mode/request` | `jetpilot_msgs/msg/OperationModeRequest` | JPBB-01のCH3状態をPROPO/AUTOへ反映 |
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
| `flags bit 1` | ROS側AUTO要求。STM32側の安全条件を省略してはならない |

USBでは正規化指令を送り、STM32が実機校正値を使って1000〜2000 µsのPWMへ変換します。これにより安全PWM、出力制限、watchdogを基板側で完結できます。

### STM32からJetson

```text
JPB1,S,sequence,rx1_us,rx2_us,rx3_us,servo_us,esc_us,vbec_mv,selector,active_path,fault_bits,CRC16
```

| Field | Definition |
| --- | --- |
| `selector` | 0=PROPO、1=AUTO |
| `active_path` | 0=DISABLED、1=MANUAL、2=AUTO、3=FAILSAFE |
| `fault_bits` | ファームウェア仕様で定義するbit mask。0はfaultなし |

## 安全境界

ROS側の`AUTO_REQUEST`だけで`SAFE_AUTO_SEL`をHighにしてはいけません。STM32は少なくともCH3、RC入力、USB heartbeat、コマンドCRCと範囲、VBEC、watchdogを独立して確認し、条件が崩れたらMANUALまたは定義済み安全状態へ遷移します。

このリポジトリはROS 2側の通信仕様を定義します。対応するSTM32実装は`jetpilot_bridge_firmware`で同じ`JPB1`仕様を実装する想定です。
