# uart_rpmsg_bus
uart based remote processor messaging bus

## How to compile
* modify setup.sh to adapt it to your own toolchain then run:
```
PC:~$ . setup.sh
PC:~$ make
```
* insert uart_rpmsg_bus.ko in your linux target:
```
Linux_target:~# insmod uart_rpmsg_bus.ko
```
## How to probe a device
to be probbed the device must be declared in the device tree.
the example bellow show how it can be declared for a STM32MP1-dk2 board. This declaration will enable the serial device present on the Arduino pins.
```
/* Uart test */
&uart7 {
	status = "okay";
	stm32f4_uart{
		compatible = "st,stm32f4-uart";
		max-speed = <57600>;
	};
};
```
