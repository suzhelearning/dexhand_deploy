"""Synthetic SDK stdout + TJVR, never opens a device."""
import socket
import struct
import sys
import time
import zlib
import signal
from tests.test_reference_manus_process import rawviz_records
from tests.test_gesture_recognition import hand_points
from tests.test_legacy_pico_palm import _packet

signal.signal(signal.SIGPIPE,signal.SIG_DFL)
sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
for sequence in range(1,10000):
    packet=bytearray(_packet(flags=0xff))
    struct.pack_into('<QQ',packet,8,sequence,1)
    for side,y in enumerate((.2,-.2)):
        struct.pack_into('<ddd',packet,44+56*side,.45,y,1.)
        for joint,p in enumerate(((0.,y,1.2),(.2,y,1.1),(.4,y,1.),(.45,y,1.))):
            struct.pack_into('<ddd',packet,204+24*(side*4+joint),*p)
    struct.pack_into('<I',packet,len(packet)-4,zlib.crc32(packet[:-4]))
    sender.sendto(packet,('127.0.0.1',int(sys.argv[1])))
    for side in ('right','left'):
        lines=rawviz_records(side,sequence,canonical_points=hand_points()).splitlines()
        print('\n'.join(lines if sequence==1 else lines[-1:]),flush=True)
    time.sleep(1/(float(sys.argv[2]) if len(sys.argv)>2 else 90))
