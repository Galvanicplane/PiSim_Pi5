#!/usr/bin/env python3
"""
pisim_json_receiver.py - Dedicated JSON Robot Config Receiver for PiSim Platform
Listens on UDP Port 7402 for live 'robot_config.json' broadcasts sent from UE5.
Parses virtual hardware ports (PWM, I2C, SPI, UART, CAM) and saves local copy.
"""

import socket
import json
import time
import os

JSON_CONFIG_PORT = 7402
LISTEN_IP = "0.0.0.0"


def start_json_receiver(port=JSON_CONFIG_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, port))

    print("\n========================================================")
    print("    PiSim JSON Robot Config Receiver - UDP Listener     ")
    print("========================================================")
    print(f"  [+] Listening for JSON Config ON : UDP Port {port}")
    print("  [+] Waiting for UE5 Garage to send robot_config.json...")
    print("========================================================\n")

    try:
        while True:
            data, addr = sock.recvfrom(65535)
            if not data:
                continue

            try:
                json_str = data.decode('utf-8')
                config = json.loads(json_str)

                robot_name = config.get("robot_name", "Unknown Robot")
                glb_path = config.get("glb_mesh_path", "")
                mass_kg = config.get("mass_kg", 0.0)
                ports = config.get("virtual_ports", [])

                print(f"\n>>> [UDP RX] ROBOT CONFIG RECEIVED from {addr[0]}:{addr[1]} <<<")
                print(f"  🤖 Robot Name  : {robot_name}")
                print(f"  📦 GLB CAD Path : {glb_path}")
                print(f"  ⚖️  Mass        : {mass_kg} kg")
                print(f"  🔌 Virtual Hardware Ports ({len(ports)} Total):")

                for port in ports:
                    p_id = port.get("port_id")
                    p_type = port.get("port_type")
                    p_target = port.get("target_component")
                    p_addr = port.get("pin_or_address")
                    p_min = port.get("min_value")
                    p_max = port.get("max_value")
                    print(f"     -> [{p_id}] Type: {p_type:<10} | Component: {p_target:<18} | Pin/Addr: {p_addr:<5} | Min/Max: ({p_min}-{p_max})")

                # Save local copy on Pi 5
                out_path = "received_robot_config.json"
                with open(out_path, "w") as f:
                    json.dump(config, f, indent=4)
                print(f"  [+] Saved local copy to: '{out_path}'\n")

            except json.JSONDecodeError:
                print(f"  [!] Received non-JSON packet ({len(data)} bytes) from {addr[0]}")
            except Exception as e:
                print(f"  [!] Error processing packet: {e}")

    except KeyboardInterrupt:
        print("\n[PiSim] JSON Config Receiver stopped.")
    finally:
        sock.close()


if __name__ == '__main__':
    start_json_receiver()
