# Doc

[Photo Gallery](https://github.com/jjmz/Atari-Quadrature-USB-Mouse-Adapter/blob/v2-platformio/doc/gallery.md)

[Reflash adapter - Linux](https://github.com/jjmz/Atari-Quadrature-USB-Mouse-Adapter/blob/v2-platformio/doc/ISP-Prog2.md)

[Reflash adapter - Windows](https://github.com/jjmz/Atari-Quadrature-USB-Mouse-Adapter/blob/v2-platformio/doc/ISP-Prog.md)

** Status LED

- LED OFF :
	Problem : CH554 not programmed, or in boot mode ?
	Boot mode is entered if the ISP pins are shorted
	It can be either P3.6 to VCC , or P1.5 to GND
	Note that P3.6 to VCC can happen if a full-speed device if connected to USB, thus the default on this project should be P1.5.

- LED ON, still :
	Program running, no recognized device on USB

- LED Blinking regularly :
	Mouse mode

- LED Pulsing : 1 pulse
	Joystick detected - default config (i.e. no specific config found)

- LED Pulsing : 2 pulses
	Joystick detected - using an entry in the joystick table


