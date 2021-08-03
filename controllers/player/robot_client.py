import socket

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 10001        # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0) as s:
    s.connect((HOST, PORT))
    # s.bind((HOST, PORT))
    welcome_message = s.recv(8)

    if welcome_message == b'Welcome\x00':
        print("I'm connected")
    elif welcome_message == b'Refused\x00':
        print("Connection refused")
    
    
    # s.listen()
    # conn, addr = s.accept()

    # with conn:
    #     print('Connected by', addr)
    #     while True:
    #         data = conn.recv(1024)
    #         if not data:
    #             break
    #         conn.sendall(data)