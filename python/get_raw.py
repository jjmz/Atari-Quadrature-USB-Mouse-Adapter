import usb.core
import usb.util
import sys

print(usb.core.show_devices())

# 044f:b315
# 0079:0006

dev = usb.core.find(idVendor=0x044f,idProduct=0xb315)

if dev is None:
    raise ValueError('Our device is not connected')

if dev.is_kernel_driver_active(0):
    dev.detach_kernel_driver(0)
usb.util.claim_interface(dev, 0)

while True:
    res=[]
    try:
        res=dev.read(0x81,8)
    except:
        print("Timeout")
    else:
        print(res)

