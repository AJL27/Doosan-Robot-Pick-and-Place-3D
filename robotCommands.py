import time
import socket


# server is opened which connects directly to the robot
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "192.168.137.50"
port = 8085
server_socket.bind((host, port))

server_socket.listen(5)
client_socket, addr = server_socket.accept()


def received_message():
    recieved_data = client_socket.recv(1024)
    print(recieved_data)


closeGripper = b"close gripper"
openGripper = b"open gripper"
moveupvar = b"49"  # move up variable
movedownvar = b"-27"  # move down variable
moveup = [b"moveup", moveupvar]
movedown = [b"movedown", movedownvar]
rotatevar = b"90"
rotategripper = [b"rotate gripper", rotatevar]


def sendmessage(message):
    client_socket.send(b"start")
    time.sleep(0.7)
    if message[0] == b'moveto':
        client_socket.send(message[0])
        time.sleep(0.7)
        # received_message() # Small delay to ensure separation

        for jtangles in message[1]:
            time.sleep(0.7)
            client_socket.send(bytes(str(jtangles), 'utf-8'))
            time.sleep(0.7)
            # received_message() # Small delay to ensure separation
        client_socket.send(b'end')
        time.sleep(0.7)
    elif message[0] == b'moveup' or message[0] == b'movedown':
        client_socket.send(message[0])
        time.sleep(0.7)
        client_socket.send(message[1])
        time.sleep(0.7)
        client_socket.send(b'end')
    elif message[0] == b'rotate gripper':
        client_socket.send(message[0])
        time.sleep(.7)
        client_socket.send(message[1])
        time.sleep(.7)
        client_socket.send(b'end')
    else:
        client_socket.send(message)
        time.sleep(0.7)
        client_socket.send(b'end')
        time.sleep(0.7)
    return 0