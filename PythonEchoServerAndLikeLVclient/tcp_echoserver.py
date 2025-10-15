import socket
import time
import random

HOST = '0.0.0.0'
PORT = 12321

def format_floats_as_string(values):
    return ' '.join(f"{v:.6f}" for v in values) + '\r\n'

def run_server():
    print(f"[Server] Listening on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        s.settimeout(1.0)  # allows checking for Ctrl+C every second

        try:
            while True:
                try:
                    conn, addr = s.accept()
                    print(f"[Server] Connected by {addr}")
                    handle_client(conn)
                except socket.timeout:
                    continue  # allow KeyboardInterrupt to raise
        except KeyboardInterrupt:
            print("\n[Server] Shutting down.")
        except Exception as e:
            print(f"[Server] Error: {e}")

def handle_client(conn):
    with conn:
        conn.settimeout(1.0)
        buffer = ""
        try:
            while True:
                try:
                    data = conn.recv(1024).decode('utf-8')
                    if not data:
                        print("[Server] Client disconnected.")
                        break

                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip().replace('\r', '')
                        try:
                            parts = line.split()
                            if len(parts) == 2:
                                floats = [float(p) for p in parts]
                                print(f"[Server] Received: {floats}")
                            else:
                                print(f"[Server] Malformed input: {line}")
                        except ValueError:
                            print(f"[Server] Could not parse: {line}")
                            continue

                        reply = format_floats_as_string(floats)
                        conn.sendall(reply.encode('utf-8'))
                        print(f"[Server] Sent: {reply.strip()}")
                except socket.timeout:
                    continue
        except (ConnectionResetError, BrokenPipeError):
            print("[Server] Connection lost.")

if __name__ == "__main__":
    run_server()
