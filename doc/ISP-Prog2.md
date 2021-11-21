Reflashing the adapter

1/ Connect Pin 2 (P1.5) with Pin 8 (Gnd) on DB-9

-> See picture (v2-isp-alt.jpg)
![v2-isp-alt](https://github.com/jjmz/Atari-Quadrature-USB-Mouse-Adapter/blob/v2-platformio/doc/v2-isp-alt.jpg)

(If 'download cfg' was not P1.5, then the pins to short are the test pins on the PCB.
See [v2-isp](https://github.com/jjmz/Atari-Quadrature-USB-Mouse-Adapter/blob/v2-platformio/doc/v2-isp.jpg) )

2/ Connect USB to USB on a computer (you need a USB Male-to-Male cable, or an adapter)

Linux detects CH554 in boot mode as idVendor=4348, idProduct=55e0

(dmesg output example)

```
[603279.266447] usb 3-9: new full-speed USB device number 7 using xhci_hcd
[603279.415593] usb 3-9: New USB device found, idVendor=4348, idProduct=55e0, bcdDevice= 2.40
[603279.415596] usb 3-9: New USB device strings: Mfr=0, Product=0, SerialNumber=0
```

3/ Use "python3 ch55xtool.py -f firmware.bin"

(ch55xtool.py included in repository - needs python usb lib)

Output :

```
jjm@local:~$ python3 ch55xtool.py -f firmware.bin 
Found CH554.
BTVER: V2.40.
Flash done.
```

Or use the Windows 'wchisptool' [ISP-Prog](https://github.com/jjmz/Atari-Quadrature-USB-Mouse-Adapter/blob/v2-platformio/doc/ISP-Prog.md)

