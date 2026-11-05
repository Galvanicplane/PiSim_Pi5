#!/usr/bin/env bash
# ==============================================================================
# pi5_quickstart.sh - Raspberry Pi 5 Hizli Baslatma Betigi
# ==============================================================================
# Kullanim:
#   bash pi5_quickstart.sh <WINDOWS_IP>
# Ornek:
#   bash pi5_quickstart.sh 192.168.1.100
# ==============================================================================

set -e

WIN_IP=$1
if [ -z "$WIN_IP" ]; then
    echo "❌ Lutfen Windows bilgisayarinizin yerel IP adresini belirtin!"
    echo "   Kullanim: bash pi5_quickstart.sh 192.168.1.100"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=================================================="
echo " 🚀 PiSim Raspberry Pi 5 Ajanlarini Baslatiyor..."
echo "    Hedef Windows IP: $WIN_IP"
echo "=================================================="

# 1. Python Zenoh kontrolu
if ! python3 -c "import zenoh" &>/dev/null; then
    echo "📦 'eclipse-zenoh' kuruluyor..."
    pip3 install --break-system-packages eclipse-zenoh || pip3 install eclipse-zenoh
fi

# 2. Pi 5 Docker Konteynerlerini ayaga kaldir (Rover 04, 05, 06)
echo "🐳 Pi 5 Docker konteynerleri baslatiliyor..."
cd "$SCRIPT_DIR/docker"
WINDOWS_IP="$WIN_IP" docker compose -f docker-compose.pi5.yml up -d --build

echo "✅ Docker konteynerleri aktif: pisim_agent_rover_04, 05, 06"
echo ""
echo "=================================================="
echo " 📋 KULLANILABILIR KOMUTLAR:"
echo "=================================================="
echo " 1. Canli Sohbete Katil:"
echo "    ZENOH_ROUTER=tcp/$WIN_IP:7447 python3 $SCRIPT_DIR/zenoh_cli_test.py"
echo ""
echo " 2. Gercek ROS 2 Koprusunu Baslat:"
echo "    ZENOH_ROUTER=tcp/$WIN_IP:7447 python3 $SCRIPT_DIR/pi5_ros2_zenoh_bridge.py"
echo ""
echo " 3. Docker Konteyner Loglarini Izle:"
echo "    docker logs -f pisim_agent_rover_04"
echo "=================================================="

