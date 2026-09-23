# Patasmonkey向け OAK-D S2 診断運用

この文書はPatasmonkeyのOAK-D S2 / OpenVINS検証走行で追加した
`depthai_driver`の診断機能を説明します。既存の画像・IMU topicの名前、型、
header stamp、QoSは変更していません。追加情報は全てDiagnosticArray topicで
出力します。

## 目的

VIOの不調を、画像特徴不足だけでなく、左右画像・IMUの時刻、sequence欠落、
露光、USB接続状態、host側の遅延から区別できるようにします。診断topicは
`pm_bag_global_localization.launch.py`のrosbag対象に含まれます。

## 診断topic

| Topic | 主な内容 | 調査に使う場面 |
|---|---|---|
| `/oak/diagnostics/left_frame`, `right_frame` | device/host同期時刻、sequence、露光、ISO、欠落数 | 左右同期、画像drop、露光変化 |
| `/oak/diagnostics/color_frame`, `depth_frame` | RGB/depthの同等metadata | RGB-D経路と意図的なrate変換の確認 |
| `/oak/diagnostics/imu_packet` | accel/gyro個別時刻・sequence・時刻差・batch位置 | 画像–IMU同期、IMU欠落の確認 |
| `/oak/diagnostics/device_info` | MX ID、IMU型、firmware、USB速度、DepthAI版 | 接続個体・USB 3.x・実機差異の記録 |

各DiagnosticArrayは一つのstatusを持ち、`schema_version=1`の安定したkey schemaを
使います。整数時刻とsequenceは文字列化されますが、浮動小数へ丸めません。

## 時刻と欠落の解釈

- `device_timestamp_ns`：OAK内部のmonotonic clock。Unix時刻ではありません。
- `host_synced_timestamp_ns`：DepthAIがhost clockへ同期した時刻です。
- `host_receive_time_ns`：driverがmetadataを生成したROS clock時刻です。
- `sequence_gap`：前回からこのdriverがpublishしなかったsequence数です。

`sequence_gap > 0`だけで、センサ内drop、USB転送、host queue上書き、ROS処理遅延を
特定することはできません。左右frameのdevice時刻差、device/host時刻間隔、IMUの
accel/gyro差、host受信時刻を合わせて判断してください。depthは`mono_fps`で生成し
`rgb_fps`でpublishするため、そのsequence skipは意図的なrate変換です。

## OAK-D S2のUSB確認

接続中の機体、MX ID、USB速度を確認します。streamは開始しません。

```bash
python3 tools/check_oak_usb_speed.py
```

期待する`usb_speed`はUSB 3.xの`SUPER`相当です。接続がUSB 2.0相当の場合、
画像・IMUの欠落、VIO更新遅れ、帯域不足の原因になり得ます。

## 起動後の検証

OAK nodeを起動した検証走行中に、次を実行します。

```bash
python3 tools/validate_oak_diagnostics.py --duration 10
```

このツールはtopic schema、元画像/IMUとmetadataのheader一致、周波数、sequence gap、
左右device時刻差をJSONで報告します。VIOを動かすこと自体や車体を動かすことは
ありません。

より詳細なkey一覧とDepthAI 2.30の露光・gain API上の制約は
[README_diagnostics.md](README_diagnostics.md)を参照してください。
