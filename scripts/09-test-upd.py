import socket
import struct
import time

def publish_float(lin_x, lin_y, ang_z):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # IP and port of the receiver
    receiver_ip = '127.0.0.1'  # Replace with the actual IP of the receiver
    receiver_port = 5006  # Replace with the actual port of the receiver

    while True:
        try:
            # Pack the float into a binary format
            data = struct.pack('!fff', lin_x, lin_y, ang_z)

            # Send the data
            sock.sendto(data, (receiver_ip, receiver_port))

            # Wait for some time before publishing the next value
            time.sleep(0.02)  # Adjust the sleep duration as needed
        except KeyboardInterrupt:
            print("Publisher stopped by user.")
            break

    # Close the socket
    sock.close()

# Example usage
publish_float(0.5, 0.0, 0.0)