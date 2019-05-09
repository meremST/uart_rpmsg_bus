_There is no guarante that this code work as a module and it should probalby directly added and compiled with the kernel_
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
