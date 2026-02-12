#!/usr/bin/env python3
"""
UDP Server for testing STM32 UDP Client

This script creates a UDP server on your PC that:
1. Listens for messages from STM32 UDP client
2. Displays received messages
3. Sends responses back to STM32

Usage:
    sudo python3 udp_server_pc.py

Make sure to:
- Run with sudo (for port binding)
- Update UDP_IP to match your PC's IP address
- STM32 must be configured to send to this IP:PORT
"""

import socket
import sys
from datetime import datetime

# ============================================================================
# CONFIGURATION - Update these to match your setup
# ============================================================================
UDP_IP = "10.4.90.58"      # Your PC's IP address (must match your network)
UDP_PORT = 31              # Port that STM32 sends to (must match udp_client.c)
BUFFER_SIZE = 1024         # Maximum receive buffer size

# ============================================================================
# Server Implementation
# ============================================================================

def main():
    print("=" * 60)
    print("UDP SERVER FOR STM32 CLIENT TESTING")
    print("=" * 60)
    print(f"Server IP:   {UDP_IP}")
    print(f"Server Port: {UDP_PORT}")
    print("=" * 60)
    
    # Create UDP socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("[INFO] UDP socket created successfully")
    except socket.error as e:
        print(f"[ERROR] Failed to create socket: {e}")
        sys.exit(1)
    
    # Bind socket to address and port
    try:
        sock.bind((UDP_IP, UDP_PORT))
        print(f"[SUCCESS] Socket bound to {UDP_IP}:{UDP_PORT}")
    except socket.error as e:
        print(f"[ERROR] Failed to bind socket: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure you're running with sudo")
        print("2. Check if another process is using port 12")
        print("3. Verify the IP address is correct")
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("SERVER READY - Waiting for messages from STM32...")
    print("=" * 60)
    print("\nPress Ctrl+C to stop\n")
    
    packet_count = 0
    
    # Main server loop
    try:
        while True:
            # Receive data from client
            data, addr = sock.recvfrom(BUFFER_SIZE)
            packet_count += 1
            
            # Get timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # Display received packet
            print("\n" + "=" * 60)
            print(f"PACKET #{packet_count} RECEIVED")
            print("=" * 60)
            print(f"Timestamp:    {timestamp}")
            print(f"From IP:      {addr[0]}")
            print(f"From Port:    {addr[1]}")
            print(f"Data Length:  {len(data)} bytes")
            print("\n--- DATA RECEIVED ---")
            try:
                decoded_data = data.decode('utf-8')
                print(decoded_data)
            except UnicodeDecodeError:
                print(f"(Binary data): {data.hex()}")
                decoded_data = "(binary)"
            print("--- END OF DATA ---")
            
            # Create response
            response = f"Server received: {decoded_data}"
            response_bytes = response.encode('utf-8')
            
            # Send response back to client
            print(f"\n--- SENDING RESPONSE ---")
            print(response)
            print("--- END OF RESPONSE ---")
            
            sock.sendto(response_bytes, addr)
            print(f"\n[SUCCESS] Response sent ({len(response_bytes)} bytes)")
            print("=" * 60)
            
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("SERVER STOPPED BY USER")
        print("=" * 60)
        print(f"Total packets received: {packet_count}")
        print("=" * 60)
    
    finally:
        sock.close()
        print("\n[INFO] Socket closed")
        print("[INFO] Server shutdown complete")


if __name__ == "__main__":
    # Check if running with sufficient privileges on Linux/Mac
    import os
    if os.name != 'nt' and os.geteuid() != 0:
        print("\n" + "!" * 60)
        print("WARNING: Not running as root/sudo")
        print("!" * 60)
        print("On Linux/Mac, you may need to run with sudo:")
        print(f"  sudo python3 {sys.argv[0]}")
        print("\nPorts below 1024 require root privileges.")
        print("Port 12 is below 1024, so sudo is required.")
        print("!" * 60)
        print("\nAttempting to continue anyway...\n")
    
    try:
        main()
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        sys.exit(1)


"""
============================================================================
                        EXPECTED OUTPUT EXAMPLE
============================================================================

When STM32 sends "Hello World", you should see:

==============================================================
UDP SERVER FOR STM32 CLIENT TESTING
==============================================================
Server IP:   10.4.90.58
Server Port: 12
==============================================================
[INFO] UDP socket created successfully
[SUCCESS] Socket bound to 10.4.90.58:12

==============================================================
SERVER READY - Waiting for messages from STM32...
==============================================================

Press Ctrl+C to stop


==============================================================
PACKET #1 RECEIVED
==============================================================
Timestamp:    2026-02-11 14:30:45.123
From IP:      10.4.90.100
From Port:    1100
Data Length:  11 bytes

--- DATA RECEIVED ---
Hello World
--- END OF DATA ---

--- SENDING RESPONSE ---
Server received: Hello World
--- END OF RESPONSE ---

[SUCCESS] Response sent (28 bytes)
==============================================================

============================================================================
                        TROUBLESHOOTING
============================================================================

If you get "Permission denied" error:
    → Run with sudo: sudo python3 udp_server_pc.py

If you get "Address already in use" error:
    → Port 12 is being used by another process
    → Find and kill it: sudo lsof -i :12
    → Or change the port in both this script and udp_client.c

If you don't receive any packets:
    → Check firewall: sudo ufw allow 12/udp
    → Check if STM32 and PC are on same network
    → Ping STM32 from PC: ping 10.4.90.100
    → Verify IP addresses match in both PC and STM32 code

If packets arrive but STM32 doesn't receive response:
    → Check STM32 firewall/routing
    → Verify STM32 has correct PC IP (10.4.90.58)
    → Use Wireshark to see if response leaves PC

============================================================================
"""
