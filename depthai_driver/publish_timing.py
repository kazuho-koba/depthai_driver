"""診断有効時のみ、publishとpollの時間を有界メモリへ保存する。"""
import csv
import json
import signal
import threading
import time
from collections import deque
from pathlib import Path


class TimingBuffer:
    """単一executor用。記録中のファイルI/Oと追加ROS購読を避ける。"""
    def __init__(self, path, capacity=150000):
        # 診断も有限RAMを使う。0で履歴なし、負数で無制限のような曖昧な設定を
        # 受け入れず、長時間試験だけ明示的に容量を選ぶ。超過はevictedに残す。
        if not isinstance(capacity, int) or isinstance(capacity, bool) or capacity <= 0:
            raise ValueError('timing容量は正の整数sample数で指定してください')
        self.path = Path(path)
        self.rows = deque(maxlen=capacity)
        self.evicted = 0

    def add(self, stage, start, duration, count=0, discarded=0, age=None):
        if len(self.rows) == self.rows.maxlen:
            self.evicted += 1
        self.rows.append((stage, start, duration*1000, count, discarded, age, self.evicted))

    def save(self):
        """停止処理の有限CSV書込みだけを重複SIGINTから保護し、完了を別ファイルに記録する。

        process groupへのSIGINTに加えてlaunchから子へSIGINTが届くと、finally内の
        CSV書込みもKeyboardInterruptで切れる。既に停止中の主threadだけで一時的に
        SIGINTを無視し、例外時も元のhandlerを戻す。SIGTERMや通常動作は変更しない。
        主thread以外ではsignal handlerを変更できないので、保護を適用しない。
        """
        marker = self.path.with_suffix(self.path.suffix+'.complete.json')
        if marker.exists():
            raise FileExistsError(str(marker))
        previous_handler = None
        if threading.current_thread() is threading.main_thread():
            previous_handler = signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('x', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['stage','start_s','duration_ms','count','discarded','header_age_ms','evicted'])
                writer.writerows(self.rows)
            # CSVをcloseできた場合だけ完了印を作る。markerのないCSVは部分保存の
            # 可能性がある。これは保存完了であり、evicted=0や全期間取得の保証ではない。
            with marker.open('x') as stream:
                json.dump(dict(schema_version=1, saved_rows=len(self.rows),
                               evicted=self.evicted, capacity=self.rows.maxlen), stream)
                stream.write('\n')
        finally:
            if previous_handler is not None:
                signal.signal(signal.SIGINT, previous_handler)


class TimedPublisher:
    """実publisherへ委譲し、topic・型・QoSを一切変えず所要時間を測る。"""
    def __init__(self, publisher, timing):
        self.publisher = publisher
        self.timing = timing

    def publish(self, message):
        start = time.perf_counter()
        age = None
        if hasattr(message, 'header'):
            stamp = message.header.stamp
            age = (time.time()-stamp.sec-stamp.nanosec/1e9)*1000
        try:
            self.publisher.publish(message)
        finally:
            self.timing.add(self.publisher.topic_name, start, time.perf_counter()-start, age=age)

    def __getattr__(self, name):
        return getattr(self.publisher, name)
