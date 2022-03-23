FROM python:3-alpine

ENV IO_METRICS_HOST=localhost \
    IO_METRICS_UDP_PORT=8092 \
    IO_METRICS_HTTP_PORT=8093

COPY requirements.txt /app/
RUN pip install -r /app/requirements.txt
COPY metrics.py /app
COPY main.py /app

CMD ["python", "/app/main.py"]