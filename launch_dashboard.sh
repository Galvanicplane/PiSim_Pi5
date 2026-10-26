#!/bin/bash
# PiSim Tactical Ground Control Station - 1-Click Dashboard Launcher for Raspberry Pi 5

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$DIR"

echo "========================================================"
echo "    PiSim Studio - Raspberry Pi 5 Web Dashboard GCS     "
echo "========================================================"

# Check if python3 is available
if ! command -v python3 &> /dev/null; then
    echo "[-] HATA: python3 bulunamadı. Lütfen 'sudo apt install python3' komutunu çalıştırın."
    exit 1
fi

# Run the web dashboard
python3 pisim_web_dashboard.py "$@"
