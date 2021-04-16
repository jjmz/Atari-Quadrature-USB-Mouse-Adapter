DataFlash Offset:

    #0 (x02) - mouse DPI divider / TODO : bit 7 for Atari/Amiga switch
    #1 (x03) - step multiplier
    #2 (x08) - minimal increment
    #3 (x20) - maximal increment

    #4 - start of joystick table

Joystick Table

    #0 (0x0F) - offset to next table entry / 0xFF for last

    #1-2 (0x0000) - Vendor ID (0 for default entry)
    #3-4 (0x0000) - Product ID (0 for default entry)

    #5-6 (0x11-0x40) condition/value for Up
    #7-8 (0x21-0xC0) condition/value for Down
    #9-A (0x10-0x40) condition/value for Left
    #B-C (0x20-0xC0) condition/value for Right
    #D-E (0x33-0xFF) condition/value for Fire Button 1 (more than 1 joystick button will trigger Fire)

Conditions encoding :

    0x1A-0xBB -> HID value at index A Lower than BB
    0x2A-0xBB -> HID value at index A Greater than BB
    0x3A-0xBB -> (value and BB(mask)) is not zero

Some examples :

- DragonRise - 0x0079 / 0x0006 - X,Y @ 0x01/0x00, buttons @ 0x06

    0x0F, 0x00,0x79,0x00,0x06, 0x11,0x40,0x21,0xC0, 0x10,0x40,0x20,0xC0, 0x36,0xFF,	

- 8bitdo / SN30 Pro - 0x2DC8 / 0x6001 - X,Y @ 0x04/0x03, buttons @ 0x00

    0x0F, 0x2D,0xC8,0x60,0x01, 0x14,0x40,0x24,0xC0, 0x13,0x40,0x23,0xC0,	0x30,0xFF,

- Logitech ATK3 - ???? ???? - X,Y @ 0x01/0x00, buttons @ 0x03

    cf Default Config

DataFlash can be programmed independantly from the firmware code using the WCHISPTOOL.EXE
(Have not found a Linux program to modify DataFlash area)    

