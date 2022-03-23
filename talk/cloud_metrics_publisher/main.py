#!/bin/env python3

import os
import random
import time

from metrics import HTTPMetricsCollector

IO_METRICS_HOST = os.environ['IO_METRICS_HOST']
IO_METRICS_UDP_PORT = int(os.environ['IO_METRICS_UDP_PORT'])
IO_METRICS_HTTP_PORT = int(os.environ['IO_METRICS_HTTP_PORT'])


if __name__ == '__main__':
    # udp_sender sends metrics over UDP
    udp_sender = HTTPMetricsCollector('udp', host=IO_METRICS_HOST, port=IO_METRICS_UDP_PORT)
    # http_sender sends metrics over HTTP
    http_sender = HTTPMetricsCollector('http', host=IO_METRICS_HOST, port=IO_METRICS_HTTP_PORT)

    turn = 0
    try:
        while True:
            # Generate random HTTP request duration data
            data = dict(
                duration=random.uniform(0.0, 11.0),
                method=random.choice(['GET', 'POST']),
                path=random.choice(['/api/path1', '/api/path2']),
                code=random.choice(['200', '201', '400', '401', '500']),
            )
            print('Sending data:', data)
            if turn == 0:
                udp_sender.collect(**data)
            else:
                http_sender.collect(**data)
            turn = 1 - turn  # keep alternating between UDP and HTTP
            time.sleep(10)
    except KeyboardInterrupt:
        udp_sender.shutdown()
        http_sender.shutdown()
