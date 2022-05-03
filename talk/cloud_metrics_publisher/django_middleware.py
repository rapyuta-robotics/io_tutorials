import time

from metrics import HTTPMetricsCollector

IO_METRICS_HOST, IO_METRICS_UDP_PORT = '', 0
# - Define IO_METRICS_HOST, IO_METRICS_UDP_PORT, and IO_METRICS_HTTP_PORT in your settings.py
#   For instance you can keep:
#     IO_METRICS_HOST = os.environ['IO_METRICS_HOST']
#     IO_METRICS_UDP_PORT = int(os.environ['IO_METRICS_UDP_PORT'])
#     IO_METRICS_HTTP_PORT = int(os.environ['IO_METRICS_HTTP_PORT'])
# - Update MIDDLEWARE list in settings.py to include:
#     '<app>.middleware.HTTPMetricsMiddleware',
# - Uncomment the following line and update path.to:
#     from path.to.settings import IO_METRICS_HOST, IO_METRICS_UDP_PORT


class HTTPMetricsMiddleware:
    def __init__(self, get_response):
        self.get_response = get_response
        self.collector = HTTPMetricsCollector('udp', IO_METRICS_HOST, IO_METRICS_UDP_PORT)

    def __call__(self, request):
        start_time = time.time()
        response = self.get_response(request)
        self.collector.collect(duration=time.time() - start_time,
                               method=request.method, path=request.path,
                               code=response.status_code)
        return response
