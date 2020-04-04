# ULX2S / ULX3S JTAG programmer usage:

```
ULX2S / ULX3S JTAG programmer v 3.0.92  
Usage: ujprog [option(s)] [bitstream_file]

 Valid options:
  -p PORT       Select USB JTAG / UART PORT (default is 0)
  -P TTY        Select TTY port (valid only with -t or -a)
  -j TARGET     Select bitstream TARGET as SRAM (default) or FLASH (XP2 only)
  -f ADDR       Start writing to SPI flash at ADDR, optional with -j flash
  -s FILE       Convert bitstream to SVF FILE and exit
  -r            Reload FPGA configuration from internal Flash (XP2 only)
  -t            Enter terminal emulation mode after completing JTAG operations
  -b SPEED      Set baudrate to SPEED (300 to 3000000 bauds)
  -e FILE       Send and execute a f32c (MIPS/RISCV) binary FILE
  -x SPEED      Set binary transfer speed, optional with -e
  -a FILE       Send a raw FILE
  -d            debug (verbose)
  -D DELAY      Delay transmission of each byte by DELAY ms
  -q            Suppress messages
```

# Compiling

Unless regularly compiling for different targets, consider copying or
symlinking the respective `Makefile.[target]` to your `Makefile` and
use just "make" to compile, for example

`ln -s Makefile.linux Makefile`

`make`


## BSD

`make -f Makefile.bsd`


## Linux PC (i386/amd64)
Native linux Debian/Ubuntu, maybe many others too,
including WSL Ubuntu, but reminder there's no WSL support for USB devices, only tty!

`make -f Makefile.linux`


## Linux RaspberryPI-3 (armhf)

Edit Makefile.linux and enable this:
`ARCHNAME = arm-linux-gnueabihf`

`make -f Makefile.linux`


## OSX

`make -f Makefile.osx`


## Windows

`make -f Makefile.win`


## MinGW (Windows 32 bit target exe; cross compiled from linux)

To get 32-bit environment on debian while running on 64-bit linux
it might be useful to enable multiarch
(but I'm not sure if this is really necessary)

`dpkg --add-architecture i386`

`apt-get update`

compiled with `i686-w64-mingw32-gcc` (installed with `apt-get install gcc-mingw-w64`); this uses the same `ftd2xx.lib` as used for linux (CDM v2.12.28 WHQL Certified\i386\ftd2xx.lib)

`make -f Makefile.ming32`

You may need to disable windows security block

![windows security block](/ujprog/images/securityblock.png)


## MinGW (Windows 64 bit target exe; cross compiled from linux)

compiled with `x86_64-w64-mingw32-gcc` (installed with `sudo apt-get install mingw-w64`)

Note this uses the 64bit `ftd2xx.amd64.lib` (`CDM v2.12.28 WHQL Certified\amd64\ftd2xx.lib`)

`make -f Makefile.ming32_64`


# NOTE on Windows Drivers

The JTAG features of this ujprog cannot be used concurrently with OpenOCD.

In order to use OpenOCD with the ULX3S, the `libusbK` dirvers are needed. One way of manually changing the drivers is to use [Zadig](https://zadig.akeo.ie/). The problem with using the `libusbK` drivers is that this ujprog will no longer work, as it needs the FTDI drivers.

## Change ULX3S Driver to libusbK using Zadig
The ULX3S is using the FTDI drivers if it shows up in the Device Manager - Ports (COM & LPT)

![ULX3S-as-FTDI-device](/ujprog/images/ULX3S-as-FTDI-device.PNG)

Launch Zadig and click on `Options - List all Devices`.  Select the ULX3S device from the dropdown:

![Zadig-FTDI-to-libusbK](/ujprog/images/Zadig-FTDI-to-libusbK.PNG)

Press the Replace Driver button and after a few moments you should see a message that the drivers were installed successfully:

![Zadig-success](/ujprog/images/Zadig-success.PNG)

The driver change typically works immediately and no reboot is needed.

## Change ULX3S Driver to FTDI 

The FTDI drivers should already be installed. If so, Windows will automatically use these drivers when a new ULXS3 is plugged in. If the FTDI Drivers are not installed, they can be downloaded from https://www.ftdichip.com/Drivers/D2XX.htm (the setup executable noted in the comments column may be the easiest way to install the drivers). 

The ULX3S is using the libusbK drivers if it shows up in Device Manager - libusbK USB Devices. (typically when using OpenOCD)

![ULX3S-as-libusbK-device](/ujprog/images/ULX3S-as-libusbK-device.PNG)

To remove the libusbK drivers, right click on your ULX3S device in Device Manager and select `Uninstall Device`:

![Uninstall-libusbK-device](/ujprog/images/Uninstall-libusbK-device.PNG)

Then click the Uninstall button (don't check the box unless you want to actually uninstall the drivers from Windows and then reinstall the drivers later; we are only uninstalling the device):

![Uninstall-libusbK-device-step2](/ujprog/images/Uninstall-libusbK-device-step2.PNG)

After clicking the Uninstall button, `Device Manager` may flicker a bit, but no message is typically shown. If the device was removed it will no longer be visible. If there are no other libusbK devices, then then entire `libusbK USB Devices` container will also be gone.

To complete the process of installing the FDTI drivers: Unplug the ULX3S, wait 30 seconds and plug it back in. Windows should automatically use the FTDI drivers and a new COM port will appear in `Device Manager - Ports (COM & LPT)` as shown above.

# Troubleshooting

*** WINDOWS ***

Most issues come from windows platform. In some cases
ujprog doesn't work. Sometimes it is just a matter of dynamic
linking (DLL file "ftd2xx.dll" or "ftd2xx64.dll", easiest is
just to copy this file from FTDI D2XX CDM driver to the same
directory where ujprog.exe is)

On VoIFS there is strange problem related with ujprog.exe
compiled with mingw. ujprog.exe, if started from "wrong" directory
doesn't work. When started from "wrong" directory, ujprog.exe
will exit without printing any error or any other message while
it should print help/usage message shown on top of this page.
Possible cause of this problem is that "ftd2xx64.dll" (for win64)
is found copied to System32 directory under name "ftd2xx.dll" (for win32).

Possible solution would be to remove all ftd2xx copies and copy
ujprog.exe and dll to another directory and try again.

*** LINUX ***

Here we have much better success, ujprog is statically linked and
doesn't depend on any other file. Most issues come from user permissions
so ujprog should be either run as root or the user should be given
permissions to access USB and serial device, that's usually done
easily with some udev rule:

    # /etc/udev/rules.d/80-fpga-ulx3s.rules
    # this is for usb-serial tty device
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
      MODE="664", GROUP="dialout"
    # this is for ujprog libusb access
    ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
      GROUP="dialout", MODE="666"

*** APPLE ***

There can be many problems, I don't know what to do
one of the issues is that ujprog executable may needs
some dynamic linked library of specific version like libusb


# Changes by gojimmypi Feb 13 2019:

Added this README.md

Microsoft dumpbin reports 51E00677 time/date Fri Jul 12 06:36:55 2013 for current repo `ftd2xx.lib` 

The most recent `ftd2xx.lib` in 2.12.28 is 599AE440 time/date Mon Aug 21 06:46:40 2017

See https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.28%20WHQL%20Certified.zip on https://www.ftdichip.com/Drivers/D2XX.htm

`Makefile.ming32` and `Makefile.ming32_64` have a rule to
automatically download (`CDM v2.12.28 WHQL Certified.zip`) and unzip
library required to link i386/amd64 exe.

`Makefile.ming32` compiles with `i686-w64-mingw32-gcc` but it's not yet
tested.


