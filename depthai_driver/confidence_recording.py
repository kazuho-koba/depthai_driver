"""DepthAI設定を、SDK固有objectを残さず診断JSONへ保存する補助処理。"""
import json


def configuration_dict(value, depth=0):
    """公開data属性を再帰保存する。未知型は文字列とし、完全保存と偽らない。

    pybindの設定は通常のdictではない。関数・class・propertyの派生data bufferは
    除外し、algorithmControl/postProcessing等の実際の設定値を取得する。
    """
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (list, tuple)):
        return [configuration_dict(v, depth+1) for v in value]
    if isinstance(value, dict):
        return {str(k): configuration_dict(v, depth+1) for k, v in value.items()}
    if hasattr(value, 'name') and hasattr(value, 'value'):
        return {'name': str(value.name), 'value': int(value.value)}
    if depth > 8:
        return {'unserialized_type': type(value).__name__, 'repr': str(value)}
    result = {}
    for name in dir(value):
        if name.startswith('_') or name in ('data', 'ts', 'tsDevice', 'sequenceNum'):
            continue
        attribute = getattr(value, name)
        if callable(attribute):
            continue
        result[name] = configuration_dict(attribute, depth+1)
    return result or {'unserialized_type': type(value).__name__, 'repr': str(value)}


def snapshot(device, pipeline, stereo, dai, parameters):
    """接続時に一度だけEEPROMとpipelineを読む。hostで校正値を上書きしない。"""
    calibration = device.readCalibration()
    def camera(socket):
        return {
            'K': calibration.getCameraIntrinsics(socket, 640, 400),
            'D': calibration.getDistortionCoefficients(socket),
        }
    data = {
        'schema_version': 1, 'mx_id': str(device.getMxId()),
        'depthai_version': dai.__version__,
        'depthai_commit': str(getattr(dai, '__commit__', 'unavailable')),
        'firmware_identity': 'DepthAI SDK bundled firmware; separate firmware hash unavailable',
        'parameters': parameters,
        'eeprom': calibration.eepromToJson(),
        'pipeline': pipeline.serializeToJson(),
        'initial_config': configuration_dict(stereo.initialConfig.get()),
        'maximum_disparity': stereo.initialConfig.getMaxDisparity(),
        'geometry': {
            'left': camera(dai.CameraBoardSocket.LEFT),
            'right': camera(dai.CameraBoardSocket.RIGHT),
            'rgb': camera(dai.CameraBoardSocket.RGB),
            'right_rectification': calibration.getStereoRightRectificationRotation(),
            'left_rectification': calibration.getStereoLeftRectificationRotation(),
            # SDK外部パラメータのtranslationはcm。解析側でmへ一度だけ換算する。
            'rgb_to_right_cm': calibration.getCameraExtrinsics(
                dai.CameraBoardSocket.RGB, dai.CameraBoardSocket.RIGHT),
            'depth_rgb_K': calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.RGB, 640, 400, keepAspectRatio=True),
            'confidence_reference': 'rectified_right_assumption_requires_device_validation',
            'disparity_reference': 'StereoDepth output with RGB alignment; not native confidence coordinates',
        },
        'limitations': [
            'Geometry mapping requires validation on the installed device/firmware.',
            'Confidence is before alignment/LR check/postprocessing; it is not a probability.',
        ],
    }
    # EEPROM JSONはSDKによってdict/stringのどちらもあり得る。
    for key in ('eeprom', 'pipeline'):
        if isinstance(data[key], str):
            data[key] = json.loads(data[key])
    return data
