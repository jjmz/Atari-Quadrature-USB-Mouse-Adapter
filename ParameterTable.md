**WAS** DataFlash Offset:
**NOW** Memory Base : 0x3700 (i.e. 256 bytes before end of file)

    #0 (x02) - mouse DPI divider / TODO : bit 7 for Atari/Amiga switch
    #1 (x03) - step multiplier
    #2 (x08) - minimal increment
    #3 (x20) - maximal increment
    #4 -> #15 - to be used for buttons

    #16 - start of joystick table
        - one joystick entry => 20 bytes

Joystick Table

    #0-1 (0x0000) - Vendor ID (0 for default entry ,i.e. first one , 0xFFFF for unused)
    #2-3 (0x0000) - Product ID (0 for default entry, i.e first one)

    #4-5   (0x11-0x40) condition/value for Up
    #6-7   (0x21-0xC0) condition/value for Down
    #8-9   (0x10-0x40) condition/value for Left
    #10-11 (0x20-0xC0) condition/value for Right
    #12-13 (0x33-0xFF) condition/value for Fire Button 1 (multiple joystick buttons can trigger Fire)
    #14-15 (0xFF-0xFF) condition/value for Fire Button 2 (multiple joystick buttons can trigger Fire)
    #16-17 (0xFF-0xFF) condition/value for Fire Button 3 (multiple joystick buttons can trigger Fire)
    #18-19 (0xFF-0xFF) not used

Conditions encoding :

    0x0A-0xBB -> value at index A equals to BB
    0x1A-0xBB -> value at index A Lower than BB
    0x2A-0xBB -> value at index A Greater than BB
    0x3A-0xBB -> (value at index A and BB(mask)) is not zero
    0x4A-0xBB -> same as 0x1A (<) , but signed compare (ie 0x80 < 0x00 < 0x7F)
    0x5A-0xBB -> same as 0x2A (>) , but signed compare (ie 0x7F > 0x00 > 0x80)

Some examples :

- DragonRise - 0x0076 / 0x0006 - X,Y @ 0x01/0x00, buttons @ 0x06

    0x0F, 0x00,0x79,0x00,0x06, 0x11,0x40,0x21,0xC0, 0x10,0x40,0x20,0xC0, 0x36,0xFF,	

- 8bitdo / SN30 Pro - 0x2DC8 / 0x6001 - X,Y @ 0x04/0x03, buttons @ 0x00

    0x0F, 0x2D,0xC8,0x60,0x01, 0x14,0x40,0x24,0xC0, 0x13,0x40,0x23,0xC0, 0x30,0xFF,

- Logitech ATK3 - ???? ???? - X,Y @ 0x01/0x00, buttons @ 0x03

    cf Default Config
