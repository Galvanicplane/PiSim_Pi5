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

# Kaynak kodları kopyala
COPY vehicle_hardware.py /app/
COPY pisim_motion.py /app/
COPY zenoh_mesh.py /app/
COPY docker/agent_node.py /app/

ENV PYTHONUNBUFFERED=1

CMD ["python", "agent_node.py"]

