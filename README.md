# Radome motor controller



## CAN protocol

The Radome CAN protocol is very simple.  Send azimuth commands to address 0x1 and elevation commands to address 0x2.  Azimuth and elevation reply back from addresses 0x101 and 0x102 respectively.  Any packet sent to address 0x0 will trigger both azimuth and elevation to reply back with their current status without initiating a motor movement, sending commands will also trigger a reply back.

Even if you feed the CAN total garbage the dish will not exceed safe positions (for elevation), speeds, or acceleration.

The controllers must be updated at least every 100 ms or faster. Othewise they will safely bring the dish to a halt.

The command structure is as follows:

```
Tramsit:
// 2^24 is 360 degrees, position is unsigned
tx byte 0 = position bits 23:16
tx byte 1 = position bits 15:8
tx byte 2 = position bits 7:0

// LSB is 1 arcsecond/s = 1/1200 dps, signed
tx byte 3 = velocity bits 15:8
tx byte 4 = velocity bits 7:0

Receive:
// 2^24 is 360 degrees, position is unsigned
rx byte 0 = position bits 23:16
rx byte 1 = position bits 15:8
rx byte 2 = position bits 7:0

// LSB is 1 arcsecond/s = 1/1200 dps, signed
rx byte 3 = velocity bits 15:8
rx byte 4 = velocity bits 7:0

// LSB is 1 mA, signed
rx byte 5 = current bits 15:8
rx byte 6 = current bits 7:0

// LSB is 0.5V, unsigned
rx byte 7 = bus voltage bits 7:0
```

I recommend casting position as a `uint32_t` such that 2<sup>32</sup> is equivalent to 360 degrees.  This makes position math very nice since overflow lines up with one rotation.

We feed the dish position *and* velocity because the velocity information in order to generate very smooth trajectories.  If you want to command a position and you don't care much about timing, it's fine to send a position command with zero velocity.  If you're tracking a satellite moving a couple degrees per second across the sky on X band, the velocity really helps (empirically we were able to maintain 0.05 degrees of tracking error up to 10 dps or so)

## Firmware overview

If you decide to build the firmware, this is an STM32CubeIDE project.  You may need to enable floating point support for printf.  Define or underine azimuth or elevation in Core/Inc/main.h to set azimuth or elevation.  Encoder offsets are hardcoded in.


