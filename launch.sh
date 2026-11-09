#!/bin/bash
# PiSim Tactical Ground Control Station - 1-Click Dashboard Launcher for Raspberry Pi 5
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

echo "========================================================"
echo "    PiSim Studio - Raspberry Pi 5 Web Dashboard GCS     "
echo "========================================================"

if ! command -v python3 &> /dev/null; then
    echo "[-] HATA: python3 bulunamadı. Lütfen 'sudo apt install python3' komutunu çalıştırın."
    exit 1
fi

# Önceki asılı kalmış eski web sunucusu işlemlerini temizle (Port 8080)
pkill -f "pisim_web_dashboard.py" 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
sleep 0.2

exec python3 pisim_web_dashboard.py "$@"
