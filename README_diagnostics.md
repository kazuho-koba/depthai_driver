# OAK-D S2 VIO diagnostics

`oakd_vio_rgbd_node` publishes DepthAI metadata separately from the existing
Image and Imu messages. Existing OpenVINS-facing topic names, message types,
header stamps and QoS are unchanged.

The metadata topics use `diagnostic_msgs/msg/DiagnosticArray`. Each message has
one status entry and a stable `schema_version=1` key. Integer values are written
as base-10 strings without floating-point conversion.

## Topics

- `/oak/diagnostics/left_frame`
- `/oak/diagnostics/right_frame`
- `/oak/diagnostics/color_frame`
- `/oak/diagnostics/depth_frame`
- `/oak/diagnostics/imu_packet`
- `/oak/diagnostics/device_info`

Frame metadata contains the original DepthAI sequence number, device-monotonic
timestamp, DepthAI host-synchronized timestamp, ROS host receive time, sequence
gap, exposure time, ISO sensitivity, lens position and dimensions. Its ROS
header is identical to the corresponding published Image header.

IMU metadata contains separate accelerometer and gyroscope sequence numbers,
device and host-synchronized timestamps, their device-time delta, batch index
and sequence gaps. Its ROS header is identical to the combined Imu message,
which continues to use the accelerometer timestamp.

Device info is repeated every five seconds and records the MX ID, connected IMU
type, IMU firmware, USB speed and DepthAI runtime version. OAK-D S2 units can
contain either BNO086 or BMI270 depending on manufacturing date, so analysis
must use this recorded value rather than assuming one model.

## Interpretation

`device_timestamp_ns` is time since the OAK device monotonic-clock epoch; it is
not Unix time. `host_synced_timestamp_ns` is DepthAI's continuously synchronized
host-clock representation. `host_receive_time_ns` is the ROS clock when the
driver built the metadata.

A non-zero `sequence_gap` proves that messages in the DepthAI sequence were not
published by this driver. It does not by itself distinguish intentional rate
conversion, a sensor/device drop, USB loss, DepthAI host queue overwrite, or
host scheduling delay. In this pipeline, depth is generated at `mono_fps` but
published at `rgb_fps`, so depth skips are expected. The BNO086 can also select
different supported internal accelerometer and gyroscope rates; gyro sequence
skips in accelerometer-paced combined packets are therefore expected. These two
cases remain observable but do not raise a diagnostic warning. Compare device
timestamp intervals, host receive intervals, stereo sequences and accelerometer
gaps to localize unexpected loss.

DepthAI 2.30 exposes image exposure time and ISO sensitivity but not an
independent analog gain value on `ImgFrame`; `sensitivity_iso` is therefore the
available exposure-gain indicator.

## Live validation

With `oakd_vio_rgbd_node` running, validate topic schemas, rates, matching
source/metadata header stamps, gaps and sensor time deltas with:

```bash
python3 tools/validate_oak_diagnostics.py --duration 10
```
