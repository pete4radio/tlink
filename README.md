# tlink
blink for SSI Testing, includes freeRTOS and runs on debugger using the Raspberry Pi extension in Visual Code

you can see the output by running
screen /dev/ttyACM0 115200

It will output a dot each time it blinks the LED, and it will output an @ symbol at addresses the scan finds a device on the I2C.


..........I2C Bus Scan
   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
00 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
10 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
20 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
30 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
40 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
50 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
60 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
70 .  .  .  .  .  .  .  @  .  .  .  .  .  .  .  .


