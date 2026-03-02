#!/usr/bin/env python3
"""
TCP Server for STM32F746 TCP Client
=====================================
Interactive bidirectional communication — NO automatic pings, NO infinite loop.

HOW IT WORKS:
  1. STM32 connects to this server on port 8080
  2. Type any message in this terminal and press Enter → it is sent to STM32
  3. STM32 echoes it back → printed here
  4. No auto-replies from server → no loop

You can also use Packet Sender to send raw TCP data to the STM32's port,
but the easiest way is to just type here.
"""

import socket
import threading
import sys
from datetime import datetime

# ── Configuration ──────────────────────────────────────────────────────────────
SERVER_IP   = '0.0.0.0'
SERVER_PORT = 8080
BUFFER_SIZE = 1024
# ───────────────────────────────────────────────────────────────────────────────

# Global reference to the active client socket (only one STM32 expected)
active_client = None
active_lock   = threading.Lock()


def ts():
    return datetime.now().strftime("%H:%M:%S")


def receive_thread(sock, addr):
    """Receive data from STM32 and print it. Never sends anything back."""
    global active_client
    client_id = f"{addr[0]}:{addr[1]}"
    print(f"\n[{ts()}] STM32 connected from {client_id}")
    print("Type a message and press Enter to send it to the STM32.\n")

    try:
        while True:
            data = sock.recv(BUFFER_SIZE)
            if not data:
                print(f"[{ts()}] STM32 disconnected.")
                break
            msg = data.decode('utf-8', errors='replace').strip()
            print(f"\n[{ts()}] << STM32 says ({len(data)} bytes):\n  {msg}\n> ", end='', flush=True)
    except Exception as e:
        print(f"\n[{ts()}] Receive error: {e}")
    finally:
        with active_lock:
            active_client = None
        sock.close()
        print(f"[{ts()}] Connection closed for {client_id}")


def input_thread():
    """Read keyboard input and send it to the connected STM32."""
    global active_client
    print("Waiting for STM32 to connect before you can send...\n")

    while True:
        line = sys.stdin.readline()
        if not line:
            break
        line = line.strip()
        if not line:
            continue

        with active_lock:
            sock = active_client

        if sock is None:
            print(f"[{ts()}] No STM32 connected yet. Waiting...\n")
            continue

        msg = line + "\r\n"
        try:
            sock.sendall(msg.encode())
            print(f"[{ts()}] >> Sent to STM32: {line}\n> ", end='', flush=True)
        except Exception as e:
            print(f"[{ts()}] Send error: {e}")


def main():
    global active_client

    print("=" * 60)
    print("  STM32F746 TCP Test Server  (Interactive Mode)")
    print("=" * 60)
    print(f"  Port    : {SERVER_PORT}")
    print(f"  Control : Type a message + Enter to send to STM32")
    print(f"  Receive : Prints everything STM32 sends back")
    print(f"  Loop    : NONE — server never auto-replies")
    print("=" * 60)
    print("Press Ctrl+C to stop.\n")

    # Start keyboard-input sender thread
    threading.Thread(target=input_thread, daemon=True).start()

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((SERVER_IP, SERVER_PORT))
    server_sock.listen(1)
    print(f"[{ts()}] Listening on 0.0.0.0:{SERVER_PORT} ...")

    try:
        while True:
            client_sock, client_addr = server_sock.accept()
            with active_lock:
                if active_client is not None:
                    # Close stale connection if STM32 reconnects
                    active_client.close()
                active_client = client_sock

            t = threading.Thread(target=receive_thread,
                                 args=(client_sock, client_addr),
                                 daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        server_sock.close()
        print(f"[{ts()}] Server stopped.")


if __name__ == "__main__":
    main()

