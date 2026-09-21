#!/usr/bin/env python3

"""Read raw accelerometer and gyroscope samples for ten seconds."""

import time

import depthai as dai


def timestamp_ms(timestamp, base_timestamp):
    return (timestamp - base_timestamp).total_seconds() * 1000.0


def main():
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 100)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    output = pipeline.create(dai.node.XLinkOut)
    output.setStreamName("imu")
    imu.out.link(output.input)

    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue("imu", maxSize=50, blocking=False)
        base_timestamp = None
        count = 0
        started = time.monotonic()

        while time.monotonic() - started <= 10.0:
            for packet in queue.get().packets:
                accelerometer = packet.acceleroMeter
                gyroscope = packet.gyroscope
                accelerometer_timestamp = accelerometer.getTimestampDevice()
                gyroscope_timestamp = gyroscope.getTimestampDevice()
                if base_timestamp is None:
                    base_timestamp = min(
                        accelerometer_timestamp, gyroscope_timestamp
                    )
                print(
                    f"t_acc={timestamp_ms(accelerometer_timestamp, base_timestamp):9.3f} ms | "
                    f"acc[m/s^2] x={accelerometer.x: .4f}, "
                    f"y={accelerometer.y: .4f}, z={accelerometer.z: .4f} | "
                    f"t_gyro={timestamp_ms(gyroscope_timestamp, base_timestamp):9.3f} ms | "
                    f"gyro[rad/s] x={gyroscope.x: .4f}, "
                    f"y={gyroscope.y: .4f}, z={gyroscope.z: .4f}"
                )
                count += 1

    print(f"\nReceived IMU packets: {count}")
    print("OK")


if __name__ == "__main__":
    main()
