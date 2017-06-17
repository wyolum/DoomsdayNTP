import struct
from numpy import random
import time
import socket

UDP_IP = "192.168.1.13"
# UDP_PORT = 2390
UDP_PORT = 123
MESSAGE = "Hello, World!"
seventyYears = 2208988800L
LSB = 1./4294967296.
NTP_PACKET_BYTES = 48

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
def bytes2unix(bytes):
    return struct.unpack('>I', bytes)[0] - seventyYears

def unix2bytes(unix):
    return struct.pack('>I', unix + seventyYears)

while 1:
    msg = MESSAGE + '.' * random.randint(10, 20)
    sock.sendto(msg, (UDP_IP, UDP_PORT))
    # time.sleep(.01)
    now = time.time()
    data = sock.recv(1024)
    if len(data) >= NTP_PACKET_BYTES:
        rec_time = bytes2unix(data[32:36]) + bytes2unix(data[36:40]) * LSB
        rtc_time = bytes2unix(data[40:44]) + bytes2unix(data[44:48]) * LSB
        delta = now  - rtc_time
        tm = time.gmtime(rtc_time)
        print now,  "%02d:%02d:%02d.%03d" %(tm.tm_hour, tm.tm_min, tm.tm_sec, (rec_time % 1) * 1000), delta, bytes2unix(data[44:48]) * LSB, rtc_time - rec_time
    time.sleep(1 - time.time() % 1)
    
