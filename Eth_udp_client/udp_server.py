import socket

# Server configuration
UDP_IP = "10.4.90.58"  # Your PC's IP
UDP_PORT = 12

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP Server listening on {UDP_IP}:{UDP_PORT}")
print("Waiting for messages...\n")

while True:
    # Receive data
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr[0]}:{addr[1]}")
    print(f"Data: {data.decode('utf-8')}")
    
    # Echo back
    response = f"Server received: {data.decode('utf-8')}"
    sock.sendto(response.encode('utf-8'), addr)
    print(f"Sent response: {response}\n")
