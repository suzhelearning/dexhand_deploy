"""Fault-injection child for native hand IPC tests; never loads a device SDK."""
import struct
import sys
import time

mode=sys.argv[1]
sys.stdout.buffer.write(struct.pack('<4sBBHQ',b'TJHK',1,0,16,1))
sys.stdout.buffer.flush()
while True:
    header=sys.stdin.buffer.read(8)
    if len(header)!=8:
        break
    magic,version,flags,size=struct.unpack('<4sBBH',header)
    body=sys.stdin.buffer.read(size-8)
    if magic==b'TJHR':
        epoch,sequence,requested=struct.unpack('<QQQ',body[:24])
        completed=time.monotonic_ns()
        if mode in ('timeout','cancel'):
            time.sleep(30)
            break
        if mode=='epoch': epoch+=1
        if mode=='sequence': sequence+=1
        if mode=='future': completed+=10_000_000_000
        if mode=='requested': requested-=1
        if mode=='backward': completed=requested-1
        frame=struct.pack('<4sBBHQQQQ',b'TJHA',1,int(mode=='flags'),40,epoch,sequence,requested,completed)
        if mode=='truncated': frame=frame[:20]
        sys.stdout.buffer.write(frame)
        sys.stdout.buffer.flush()
        break
