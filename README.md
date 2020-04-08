# led_dimmer  
  

## Pin assignment  
    * GPIO5 is input - Button
    * GPIO12 is assigned as the PWM channel 0.  
    * GPIO13 is assigned as the PWM channel 1.  


### Configure the project  

```
make menuconfig
```

* Set serial port under Serial Flasher Options.


### Build and Flash  

Build the project and flash it to the board, then run monitor tool to view serial output:

```
make -j4 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

  
