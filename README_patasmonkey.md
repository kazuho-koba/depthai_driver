# Patasmonkey向け OAK-D S2 診断運用

この文書はPatasmonkeyのOAK-D S2 / OpenVINS検証走行で追加した
`depthai_driver`の診断機能を説明します。既存の画像・IMU topicの名前、型、
header stamp、QoSは変更していません。追加情報は全てDiagnosticArray topicで
出力します。

## 遠隔GUI向けの表示専用JPEG

`oak_preview_node`は原画像をBEST_EFFORT・履歴1で受け、最新1枚だけを最大5 Hz、
幅160 px、JPEG quality50で `/oak/preview/image/compressed` へ配信します。
camera本体とは別プロセスで、購読者がいない間はJPEG変換しません。受信後0.5秒を
超えた画像は捨て、header stamp/frame_idは採用した原画像を保持します。

開発PCへのraw color配信でOAK本体のpublishが秒単位で待つ現象に対する表示経路の
分離です。VIO／mapper／bag向けの原画像とQoSは変えません。低品質JPEGは検証保存用
画像ではなく人間の表示専用です。遅い遠隔送信の影響が原画像に戻らないことを実機で
確認してください。

```bash
ros2 run depthai_driver oak_preview_node
```

parameterは `input_topic`、`output_topic`、`publish_rate_hz`（0より大・30以下）、
`max_width`（1〜1920）、`jpeg_quality`（1〜100）、`max_age_sec`（正の秒数）。
pm_coreではOAKと同時起動し、`use_oak_preview:=false` で無効化できます。

## 診断の目的

VIOの不調を、画像特徴不足だけでなく、左右画像・IMUの時刻、sequence欠落、
露光、USB接続状態、host側の遅延から区別できるようにします。診断topicは
`pm_bag_global_localization.launch.py`のrosbag対象に含まれます。

## RGB-D CameraInfo

RGB-D画像には、次のCameraInfoも出力します。

| Topic | 内容 |
|---|---|
| `/oak/color/camera_info` | RGB EEPROM校正値を640×400のcolor preview（aspect ratioを保たないstretch）に合わせた内部行列・歪み係数 |
| `/oak/depth/camera_info` | 同じRGB EEPROM校正値をRGB整列depth出力（aspect ratioを保つ既定resize）に合わせた内部行列・歪み係数 |

2つの画像は同じ640×400でもresize方法が異なるため、CameraInfoの焦点距離が異なります。
後段で画素単位のcolor-depth重畳を行う場合は、画像変換のaspect ratio設定も一致させるか、
双方を共通のカメラモデルへ再投影してください。各CameraInfoのheader stampは対応する
画像のstampです。

実行中に値を確認するには、次を使います。

```bash
ros2 topic echo /oak/color/camera_info sensor_msgs/msg/CameraInfo
```

ROS 2 Foxyでは連続表示されるため、確認後に`Ctrl-C`で終了します。

CameraInfoは起動時にOAK-DのEEPROM校正値から作成されるため、publishのためにKalibrを
実行する必要はありません。

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

## 配送遅延の診断

`timing_diagnostics:=true`、`timing_csv:=<新規CSVパス>`でpollとtopic別publishの
所要時間、header age、batch省略数を有界RAMへ保持して正常停止時に保存します。
`timing_max_samples`は既定150000 sampleで、
pm_bringupでは`oak_timing_max_samples`から指定できます。長時間試験で先頭が
消えた場合はCSV末尾の`evicted`を確認し、必要な試験だけ容量を選んでください。
過去の約200秒の比較試験では300000 sampleを使用しましたが、既定値は維持しています。
sampleはmessage件数ではなくpoll/publish等の計測1回です。追加容量はRAMを使い、
センサ周波数やpublisher QoSの変更ではありません。容量0以下は診断ON時に拒否します。
診断の既定はfalseです。設定した上限を超えると古い行を破棄し、evicted欄に件数を示します。
header_age_msはROS時刻とsystem wall timeが同じ実機運用用で、sim timeには使えません。
batch内の省略はdevice内部dropとは異なり、深度の20→10Hz間引きも別に扱います。

終了時のCSV保存中だけ、主threadでは重複SIGINTを一時的に無視し、保存後に元の
handlerへ戻します。停止処理のfinallyにもSIGINTが届いてCSVが途中で切れる問題への
対策であり、通常動作やSIGTERMの扱いは変えません。
CSVをcloseできた場合だけ`<CSV名>.complete.json`を作り、`saved_rows`、`evicted`、
`capacity`を残します。markerがなければ部分保存の可能性があり、試験全体の最大値や
百分位として扱わないでください。markerがあっても容量超過・先頭除外は別に確認します。
# 共通時計変換と低頻度監視（2026-10-14）

`oakd_vio_rgbd_node` の `timestamp_mode` は既定 `host_clock`（新方式）です。
`host_clock` では最初のデータの処理待ちを時計オフセットへ混入させず、
現在の `dai.Clock.now()` の前後でROS時計を読み、最短の読み取り時間幅の組を採用します。
対応付けは起動中固定です。実行中にシステム時計・ROS時計がジャンプする場合や
`use_sim_time` のカメラ実機運用は、この方式の検証対象外です。

全モノクロ・RGB・depth・OAK IMUは1つのmapperを共有します。CameraInfoは対応画像の
Headerをコピーし、confidence/disparityは同じsequenceのdepth Headerをコピーします。
RGBとdepthの撮像時刻を強制一致させることはしません。生device/host timestamp、
露光・sequence・accel/gyro個別timestamp等の診断値は変更しません。
`/oak/diagnostics/device_info` に方式、整数nsオフセット、基準読み取り幅を追加します。
device_info自体のHeaderは状態取得時刻のためROS現在時刻のままです。

```bash
# pm_bringupの共通launchから明示ON
ros2 launch pm_bringup pm_core.launch.py oak_timestamp_mode:=host_clock
# ロールバック（再起動が必要）
ros2 launch pm_bringup pm_core.launch.py oak_timestamp_mode:=legacy_first_message
```

この例だけでは機体安全の条件を指定していません。センサ試験では追加で
`use_vehicle_interface:=false use_teleop:=false use_joy:=false` を指定してください。
旧方式でも浮動小数点の絶対時刻を避け、整数nsで計算します。旧挙動の復元は
時計対応の方式であり、旧float丸め誤差までbit単位で再現するものではありません。

## 時計のずれ監視

既存の `/oak/diagnostics/device_info` を5秒ごとに配信する際、現在のROS−DepthAI時計差を
再測定し、採用中の固定オフセットとの差を記録します。Headerのオフセットは更新しません。
timerはSTEADY_TIMEを使い、ROS時計の逆行で監視が止まることを避けます。
executor自体の停滞時には遅れるため、厳密な5秒監視や時計ジャンプの完全検出は保証しません。

| driver parameter | 既定 | 意味 |
|---|---|---|
| clock_monitor_enabled | true | falseで追加時計読み取りをOFF |
| clock_warn_threshold_ms | 5.0 | 固定オフセットとの差の絶対値が超過したらWARN |
| clock_max_reference_span_ms | 1.0 | 最短読み取り幅が超過したら測定を不確実とする |

各回も9組から最短幅を選びます。読み取り不良はREFERENCE_UNRELIABLEでWARN、
時計差超過はOFFSET_EXCEEDEDでWARN、未初期化はNOT_INITIALIZEDでSTALEです。
復帰すれば現在状態はOKに戻しますが、累積超過回数・不確実回数・観測最大絶対差を保持します。
診断keyはclock_offset_delta_ns、clock_observed_offset_ns、clock_monitor_span_ns、
clock_monitor_check_count、clock_monitor_exceeded_count、clock_monitor_unreliable_count、
clock_monitor_max_abs_delta_ns等です。delta正は現在のROS−DepthAI差が増えたことを表します。
範囲不良時はdeltaをNoneとして保存し、測定の遅れを時計異常と断定しません。
5msは暫定警告値であり、VO精度・同期精度の許容値として実証したものではありません。
DepthAI内部のdevice→host同期誤差、5秒未満で発生して復帰した時計ジャンプは検出できない場合があります。
既存mission/debug bagはdevice_infoを記録するため、追加topicやbag一覧変更は不要です。
