## CH55x Bootloader

I think you can have 3 cases when powering the CH55x :

1. Unprogrammed CH55x

When the CH55x is unprogrammed, it will by default go in bootloader mode, with no specific pin to short.

2. Factory default (after 1st flashing, if nothing is done regarding download cfg), or if P3.6 was selected

To go in bootloader mode, then a __temporary__ connection of P3.6 with VCC is needed while powering the chip.
This __SHOULD NOT__ be the default for this project, as connecting a full-speed device to the USB would
cause the CH55x to enter bootloader mode too (Full-speed devices have a pull-up connected to D+).

3. Alternative download cfg : via P1.5 [^1]

This is the configuration I am using - note that P1.5 has to be connected to GND - both pins are
present on the DB-9 connector : P1.5 is pin 2 , GND is pin 8.

## Windows WCH utility

Software to use : WCHISPTOOL ([local copy here](WCHISPTool_Setup.exe?fileId=854562))

![Wch-prog.jpg](Wch-prog.jpg?fileId=854563#mimetype=image%2Fjpeg&hasPreview=true)

Need to use a Male-Male USB cable (or adapter) :

![usb-usb.jpg](usb-usb.jpg?fileId=881106#mimetype=image%2Fjpeg&hasPreview=true)

V1-Bootloader-Pins :

![v1-isp.jpg](v1-isp.jpg?fileId=854571#mimetype=image%2Fjpeg&hasPreview=true)

V2-Bootloader-Pins (only the two test pins - forget about the white wire) - same a V1, but smaller :

![v2-isp.jpg](v2-isp.jpg?fileId=881125#mimetype=image%2Fjpeg&hasPreview=true)

V2-Alternate-Bootloader-Pins (DB9-2 (P1.5) and DB9-8 (Gnd)) :

![v2-isp-alt.jpg](v2-isp-alt.jpg?fileId=881115#mimetype=image%2Fjpeg&hasPreview=true)

Some Misc photos :

![versions.jpg](versions.jpg?fileId=881096#mimetype=image%2Fjpeg&hasPreview=true)

References :

[^1]: http://rabid-inventor.blogspot.com/2020/05/getting-started-with-ch55x.html
