# MicroBike

PCB tutorial: https://drive.google.com/file/d/1_RPSxXgNnm7xqQideJVZ_5Tqhl-yj-vk/view

## Ordering parts:
digikey cart: https://www.digikey.com/ordering/shoppingcart

## Bluetooth Notes:

The RN4671 has hardware flow control enabled by default, meaning that it requires the use of the CTS and RTS pins. We will need to disable this. See the [datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/RN4678-Bluetooth-Dual-Mode-Module-Data-Sheet-DS50002519D.pdf) page 7 for information about how. An answers page [here](https://microchip.my.site.com/s/article/Option-to-disable-flow-control-in-RN4678) says that if we connect CTS and RTS pins on the bluetooth together with a jumper temporarily, we will enter command mode. Then, we will be able to write a quick Arduino sketch to configure the RN chip to disable flow control. After that is done, we can disconnect the jumper and use the RN as normal.

Bluetooth Classic replacements: https://microchip.my.site.com/s/article/Bluetooth-Classic-replacements

https://www.digikey.com/en/products/detail/microchip-technology/RN4678-V-RM100/6221218
![image_720](https://github.com/wenjia123/MicroBike/assets/97308209/b73a5f78-1511-478f-83ba-39639f117563)

## Rear drive motor notes:

here is what the connections to the arduino chip and the motor need to be.
1. OUT1 goes to motor+ (one of the 6 pins on your motor connector)
2. OUT2 goes to motor- (another one of hte 6 pins on your motor connector)
3. EN goes to an arduino digital pin of your choice, but it MUST be one that can do PWM output (analogWrite). We flash our 32U4 chip to be the same as the Pololu Micro 32U4 board, so the pins with nets marked D5,D6,D9,D10,D11 are all good choices (the ones with the ~ mark). this pin controlls the voltage we send to the motor by pulsing it.
4.PH goes to another free arduino digital pin. This controls the direction (logic HIGH forward, logic LOW backward). This does not have to be a ~ pin.
5.VIN should be the RAW BATTERY VOLTAGE, not a regulated 3V or 5V net. this is so that the regulators will not be loaded when the rear wheel motor is used (the regulators can't provide a lot of power, but the battery can).
6.GND should be ground.
![image_720](https://github.com/wenjia123/MicroBike/assets/97308209/1fb80222-05cc-4677-ae95-f00f1730f2a4)

https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
https://www.pololu.com/product/3101

![image_720](https://github.com/wenjia123/MicroBike/assets/97308209/48019868-a396-4c36-8dcc-5d5a926bcc95)







<img width="915" alt="Screenshot 2024-01-13 at 12 38 53 PM" src="https://github.com/wenjia123/MicroBike/assets/97308209/97381264-6f78-4b70-a4b5-66e887361bce">

FUse for the bettery switch: https://www.digikey.com/en/products/detail/littelfuse-inc/NANOSMDC150F-2/1045876
Clock: https://www.digikey.com/en/products/detail/ecs-inc/ECS-160-18-33-AEN-TR/8023240
