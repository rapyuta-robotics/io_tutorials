FROM python:3.7.3-alpine
COPY requirements.txt /app/
RUN pip install -r /app/requirements.txt
COPY . /app

CMD ["python","/app/app.py"]
