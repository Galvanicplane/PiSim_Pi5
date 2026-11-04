# syntax=docker/dockerfile:1
FROM python:3.11-slim

WORKDIR /app

# Sistem bağımlılıkları
RUN apt-get update && apt-get install -y --no-install-recommends \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Python kütüphaneleri: eclipse-zenoh ve numpy
RUN pip install --no-cache-dir eclipse-zenoh numpy

# Tüm kaynak kodları kopyala
COPY . /app/

ENV PYTHONUNBUFFERED=1

CMD ["python", "docker/agent_node.py"]
