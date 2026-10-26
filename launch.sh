#!/bin/bash
# PiSim 1-Click Launcher shortcut
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
exec "$DIR/launch_dashboard.sh" "$@"
