import socket

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 14541))
    print("Listening on UDP port 14541...")
    for _ in range(5):
        data, addr = sock.recvfrom(1024)
        print(f"Received {len(data)} bytes from {addr}")

if __name__ == "__main__":
    main()
