#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
agent_node.py - PiSim Docker Konteyner Ajan Giriş Noktası
=========================================================
Bu dosya, Docker konteyneri ayağa kalktığında çalışan ana giriş noktasıdır.
Tüm otopilot, bağlantı denetimi ve görev yönetimi mantığını `agent_supervisor`
modülü üzerinden koordine eder.
"""

import os
import sys

# Dizin yollarını sys.path'e ekle
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
for p in (PARENT_DIR, SCRIPT_DIR):
    if p and p not in sys.path:
        sys.path.insert(0, p)

from agent_supervisor import main

if __name__ == "__main__":
    main()
