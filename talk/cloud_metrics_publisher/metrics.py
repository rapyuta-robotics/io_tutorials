import json
import traceback
from dataclasses import dataclass
from queue import Queue
from threading import Thread
from typing import Dict, Optional

from telegraf.client import TelegrafClient
import requests

BUCKETS = [0.01, 0.05, 0.1, 0.5, 1, 5, 10]


@dataclass
class Metric:
    metric_group: str
    fields: Dict[str, float]
    tags: Optional[Dict[str, str]]

    def to_json(self):
        m = {'name': self.metric_group, 'fields': self.fields}
        if self.tags:
            m['tags'] = self.tags
        return json.dumps(m)


@dataclass
class _Stat:
    duration: float
    method: str
    path: str
    code: int


class HTTPMetricsCollector:
    QUEUE_END = 'QUEUE_END'

    def __init__(self, send_protocol: str, host: str, port: int):
        if send_protocol == 'udp':
            self.send = self.udp_send(host, port)
        elif send_protocol == 'http':
            self.send = self.http_send(host, port)
        else:
            raise Exception('send_protocol must be one of [http, udp]')
        self.queue = Queue()
        self.thread = Thread(target=self.consumer)
        self.thread.start()

    @staticmethod
    def udp_send(host, port):
        udp_client = TelegrafClient(host=host, port=port)

        def send(metric: Metric):
            udp_client.send(metric.to_json())
        return send

    @staticmethod
    def http_send(host, port):
        http_client = requests.Session()
        url = f'http://{host}:{port}/io_metrics'

        def send(metric: Metric):
            http_client.post(url, data=metric.to_json(), timeout=5)
        return send

    @staticmethod
    def get_bucket(duration):
        prev = 0
        for bucket in BUCKETS:
            if duration <= bucket:
                return f'{prev}-{bucket}'
            prev = bucket
        return f'{prev}-Inf'

    def consumer(self):
        while True:
            stat = self.queue.get(block=True)
            if stat == self.QUEUE_END:
                self.queue.task_done()
                break
            metric = Metric('http_duration',
                            fields={'seconds': round(stat.duration, 3)},
                            tags=dict(method=stat.method, path=stat.path, code=str(stat.code),
                                      bucket=self.get_bucket(stat.duration)))
            try:
                self.send(metric)
            except Exception:
                print(traceback.format_exc())
            self.queue.task_done()

    def collect(self, duration, method, path, code):
        stat = _Stat(duration, method, path, code)
        self.queue.put(stat, block=False)

    def shutdown(self):
        self.queue.put(self.QUEUE_END)
        self.queue.join()
        self.thread.join()
