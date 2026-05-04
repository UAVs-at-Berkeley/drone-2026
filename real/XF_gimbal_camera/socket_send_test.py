
import socket   
import time

hex_string = "a8e54800010000d00730f805000000000000000000000000000000000000010000000000000124f2df6516eeaa16a3a000000fb00c0615e60810270000000000000000000014e28c"
hex_bytes = bytes.fromhex(hex_string)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    while True:
        sock.sendto(hex_bytes, ("192.168.144.108", 2337))
        time.sleep(5)
except KeyboardInterrupt:
    pass
finally:
    sock.close()
