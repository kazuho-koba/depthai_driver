"""DepthAIのホスト単調時計をROS時計へ対応付ける、ROS非依存の変換。

全OAKストリームが1つの対応付けを共有し、画像–IMUの相対時刻を維持する。
起動時だけ固定オフセットを求める。実行中の時計ジャンプを黙って補正しない。
"""


def timedelta_ns(value):
    """timedeltaを整数nsへ変換し、巨大な絶対時刻のfloat丸めを避ける。"""
    return (value.days * 86400 + value.seconds) * 10**9 + value.microseconds * 1000


class ClockMapping:
    """旧方式と試験方式を起動パラメータで選ぶ。ROS・DepthAI時計は関数で注入する。"""

    def __init__(self, mode, ros_now_ns, host_now_ns):
        if mode not in ('legacy_first_message', 'host_clock'):
            raise ValueError('timestamp_modeはlegacy_first_messageまたはhost_clockです')
        self.mode = mode
        self.ros_now_ns = ros_now_ns
        self.host_now_ns = host_now_ns
        self.offset_ns = None
        self.reference_span_ns = None

    def sample_reference(self):
        """現在の時計差を測るだけで、採用済みの固定オフセットは変更しない。"""
        candidates = []
        for _ in range(9):
            before = self.host_now_ns()
            ros = self.ros_now_ns()
            after = self.host_now_ns()
            if after >= before:
                candidates.append((after-before, ros-(before+after)//2))
        if not candidates:
            raise RuntimeError('DepthAIホスト時計が逆行しました')
        return min(candidates)

    def convert(self, sample_ns):
        """初回だけ基準を測定し、以後は全データへ同じ整数オフセットを加える。"""
        if self.offset_ns is None:
            if self.mode == 'legacy_first_message':
                # ロールバック用。旧実装の「最初のデータを処理時刻へ合わせる」動作。
                self.offset_ns = self.ros_now_ns() - sample_ns
            else:
                # DepthAI時計でROS時計の読み取りを挟む。最短の組を選ぶことで、
                # OSの割込み・スケジューリング待ちの影響を抑える（待機sleepなし）。
                self.reference_span_ns, self.offset_ns = self.sample_reference()
        return sample_ns + self.offset_ns


class ClockMonitor:
    """低頻度診断用の有界状態。読み取り不良と実際の時計差超過を区別する。"""

    def __init__(self, mapping, threshold_ns, max_span_ns):
        if threshold_ns <= 0 or max_span_ns <= 0:
            raise ValueError('時計監視の閾値と読み取り幅上限は正数が必要です')
        self.mapping = mapping
        self.threshold_ns = threshold_ns
        self.max_span_ns = max_span_ns
        self.check_count = self.exceeded_count = self.unreliable_count = 0
        self.max_abs_delta_ns = 0

    def check(self):
        """診断snapshotを返す。delta正は現在のROS−DepthAI差が増えたことを表す。"""
        result = {'clock_monitor_state': 'NOT_INITIALIZED', 'clock_offset_delta_ns': None,
                  'clock_monitor_span_ns': None, 'clock_observed_offset_ns': None}
        if self.mapping.offset_ns is not None:
            self.check_count += 1
            try:
                span, observed = self.mapping.sample_reference()
                result.update(clock_monitor_span_ns=span, clock_observed_offset_ns=observed)
                if span > self.max_span_ns:
                    self.unreliable_count += 1
                    result['clock_monitor_state'] = 'REFERENCE_UNRELIABLE'
                else:
                    delta = observed-self.mapping.offset_ns
                    self.max_abs_delta_ns = max(self.max_abs_delta_ns, abs(delta))
                    exceeded = abs(delta) > self.threshold_ns
                    self.exceeded_count += int(exceeded)
                    result.update(clock_offset_delta_ns=delta,
                                  clock_monitor_state='OFFSET_EXCEEDED' if exceeded else 'OK')
            except RuntimeError:
                self.unreliable_count += 1
                result['clock_monitor_state'] = 'REFERENCE_UNRELIABLE'
        result.update(clock_monitor_check_count=self.check_count,
                      clock_monitor_exceeded_count=self.exceeded_count,
                      clock_monitor_unreliable_count=self.unreliable_count,
                      clock_monitor_max_abs_delta_ns=self.max_abs_delta_ns,
                      clock_monitor_threshold_ns=self.threshold_ns,
                      clock_monitor_max_reference_span_ns=self.max_span_ns)
        return result
