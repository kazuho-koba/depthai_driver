# OAK-D S2 VIO診断

`oakd_vio_rgbd_node`は既存のImage/Imu messageと別にDepthAI metadataをpublishする。
OpenVINS向けtopic名、message型、header stamp、QoSは変更しない。

metadata topicは`diagnostic_msgs/msg/DiagnosticArray`を使う。各messageは一つのstatusと
安定した`schema_version=1` keyを持ち、整数値は浮動小数へ変換せず10進文字列で保存する。

## Topic

- `/oak/diagnostics/left_frame`、`right_frame`：device/host同期時刻、sequence、露光、
  ISO sensitivity、lens位置、画像寸法、sequence gap。
- `/oak/diagnostics/color_frame`、`depth_frame`：RGB-Dの同等metadata。
- `/oak/diagnostics/imu_packet`：accel/gyro個別のsequence、device/host同期時刻、
  device時刻差、batch内位置、sequence gap。
- `/oak/diagnostics/device_info`：MX ID、接続IMU型、IMU firmware、USB速度、DepthAI版。

frame metadataのROS headerは対応するImageと同一である。IMU metadataのheaderは
combined Imu messageと同一で、従来どおりaccelerometer timestampを代表時刻にする。
OAK-D S2の接続IMUはBNO086またはBMI270の場合があるため、解析では仮定せず
`device_info`に記録された値を使う。

## 時刻と欠落の解釈

`device_timestamp_ns`はOAK内部monotonic clockのepochからの時刻で、Unix時刻ではない。
`host_synced_timestamp_ns`はDepthAIが同期したhost clock表現、`host_receive_time_ns`は
driverがmetadataを作ったROS clockである。

`sequence_gap`が非ゼロなら、DepthAI sequenceの一部をこのdriverがpublishしていない。
ただしsensor/device drop、USB損失、host queue上書き、host scheduling遅延を単独では
区別できない。左右frameのdevice時刻差、device/host時刻間隔、IMUのaccel/gyro差、
host受信時刻を併せて判定する。

depthは`mono_fps`で生成し`rgb_fps`でpublishするため、depthのsequence skipは想定した
rate変換である。BNO086でaccelerometerとgyroscopeの内部周波数が異なる場合、
accelerometer基準combined packetのgyro sequence skipも想定内である。

DepthAI 2.30の`ImgFrame`は独立analog gainではなくISO sensitivityを公開するため、
露光・gainの指標として`sensitivity_iso`を記録する。

## live検証

OAK node起動中にschema、周波数、元topicとmetadataのheader一致、gap、sensor時刻差を
検証する。

```bash
python3 tools/validate_oak_diagnostics.py --duration 10
```

Patasmonkey実走行での利用法とUSB速度確認は
[README_patasmonkey.md](README_patasmonkey.md)を参照する。
