[![Travis (.org)](https://img.shields.io/travis/kost/fujprog)](https://travis-ci.org/kost/fujprog "Travis CI")

# ULX2S / ULX3S JTAG programmer

This is FPGA JTAG programmer for [ULX2/3S boards](https://radiona.org/ulx3s/). You need to have bitstream ready.

## Usage

FPGA ULX2S / ULX3S JTAG programmer command line options:

```
Usage: fujprog [option(s)] [bitstream_file]

 Valid options:
  -p PORT	Select USB JTAG / UART PORT (default is 0)
  -P TTY	Select TTY port (valid only with -t or -a)
  -T TYPE	Select TYPE of input (svf, img, bit or jed)
  -i 		identify and exit
  -j TARGET	Select bitstream TARGET as SRAM (default) or FLASH
  -f ADDR	Start writing to SPI flash at ADDR, optional with -j flash
  -s FILE	Convert bitstream to SVF FILE and exit
  -r		Reload FPGA configuration from FLASH
  -t		Enter terminal emulation mode after completing JTAG operations
  -b SPEED	Set baudrate to SPEED (300 to 3000000 bauds)
  -e FILE	Send and execute a f32c (MIPS/RISCV) binary FILE
  -x SPEED	Set binary transfer speed, optional with -e
  -a FILE	Send a raw FILE
  -d 		debug (verbose)
  -D DELAY	Delay transmission of each byte by DELAY ms
  -V 		display version and exit
  -z 		Force action
  -h 		This help message
  -l X		Display messages in log fashion every <X> times
  -S serial 	Select FTDI by serial to support multiple boards
  -q 		Suppress messages
```

## Example usage

Upload bitstream:
```
fujprog bitstream.bit
```

Upload bitstream to flash:
```
fujprog -j flash bitstream.bit
```

Upload bitstream from curl:
```
curl https://www.site.com/bitstream.bit | fujprog
```

Upload flash image from curl:
```
curl https://www.site.com/bitstream.img | fujprog -T img -j flash
```

# Building

It is standard CMake procedure:

```
mkdir build
cd build
cmake ..
make
make install
```

You can also pass optional parameters:

```
cmake -DBUILD_STATIC=ON -DLIBFTDISTATIC=/opt/libftdi/lib/libftdi.a -DLIBUSB0STATIC=/opt/libusb0/lib/libusb.a ..
make install/strip
```

### Linux requirements


You need following packages in order to build on Debian/Ubuntu systems:
```
sudo apt-get install libftdi1-dev libusb-dev cmake make build-essential
```

## MacOS X build

You need to install following dependencies (tested with homebrew):
```
brew install libftdi libftdi0
```

And then it is standard build with `-DSTATICLIB=ON`:

```
mkdir build
cd build
cmake -DBUILD_STATICLIB=ON ..
make
make install/strip
```

Note that full static binary is not supported by Apple.

## Windows build

You need to download ftdi dependency lib:
```
wget https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.28%20WHQL%20Certified.zip
```

Then you can just open folder inside Visual Studio and build Release versions.

## Windows cross-compile

You need to download ftdi dependency lib:
```
wget https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.28%20WHQL%20Certified.zip
```

Cross compiling is done using standard cmake toolchain file:
```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=`pwd`/../cmake/Toolchain-cross-mingw32.cmake ..
make
```

## FreeBSD build

You need to install dependencies:
```
pkg install libftdi1 pkgconf git cmake
```

And then it is standard CMake procedure:
```
mkdir build
cd build
cmake ..
make
make install
```

If you need static binary version, you need to perform following cmake command:
```
cmake -E env CFLAGS="-pthread" cmake -DBUILD_STATIC=ON ..
```

# Windows Notes

You may need to disable windows security block

![windows security block](/images/securityblock.png)

## Windows Drivers

The JTAG features of this fujprog cannot be used concurrently with OpenOCD.

In order to use OpenOCD with the ULX3S, the `libusbK` dirvers are needed. One way of manually changing the drivers is to use [Zadig](https://zadig.akeo.ie/). The problem with using the `libusbK` drivers is that this fujprog will no longer work, as it needs the FTDI drivers.

## Change ULX3S Driver to libusbK using Zadig

The ULX3S is using the FTDI drivers if it shows up in the Device Manager - Ports (COM & LPT)

![ULX3S-as-FTDI-device](/images/ULX3S-as-FTDI-device.PNG)

Launch Zadig and click on `Options - List all Devices`.  Select the ULX3S device from the dropdown:

![Zadig-FTDI-to-libusbK](/images/Zadig-FTDI-to-libusbK.PNG)

Press the Replace Driver button and after a few moments you should see a message that the drivers were installed successfully:

![Zadig-success](/images/Zadig-success.PNG)

The driver change typically works immediately and no reboot is needed.

## Change ULX3S Driver to FTDI 

The FTDI drivers should already be installed. If so, Windows will automatically use these drivers when a new ULXS3 is plugged in. If the FTDI Drivers are not installed, they can be downloaded from https://www.ftdichip.com/Drivers/D2XX.htm (the setup executable noted in the comments column may be the easiest way to install the drivers). 

The ULX3S is using the libusbK drivers if it shows up in Device Manager - libusbK USB Devices. (typically when using OpenOCD)

![ULX3S-as-libusbK-device](/images/ULX3S-as-libusbK-device.PNG)

To remove the libusbK drivers, right click on your ULX3S device in Device Manager and select `Uninstall Device`:

![Uninstall-libusbK-device](/images/Uninstall-libusbK-device.PNG)

Then click the Uninstall button (don't check the box unless you want to actually uninstall the drivers from Windows and then reinstall the drivers later; we are only uninstalling the device):

![Uninstall-libusbK-device-step2](/images/Uninstall-libusbK-device-step2.PNG)

After clicking the Uninstall button, `Device Manager` may flicker a bit, but no message is typically shown. If the device was removed it will no longer be visible. If there are no other libusbK devices, then then entire `libusbK USB Devices` container will also be gone.

To complete the process of installing the FDTI drivers: Unplug the ULX3S, wait 30 seconds and plug it back in. Windows should automatically use the FTDI drivers and a new COM port will appear in `Device Manager - Ports (COM & LPT)` as shown above.

# Troubleshooting

##  WINDOWS

Most issues come from windows platform. In some cases
fujprog doesn't work. Sometimes it is just a matter of dynamic
linking (DLL file "ftd2xx.dll" or "ftd2xx64.dll", easiest is
just to copy this file from FTDI D2XX CDM driver to the same
directory where fujprog.exe is)

On VoIFS there is strange problem related with fujprog.exe
compiled with mingw. fujprog.exe, if started from "wrong" directory
doesn't work. When started from "wrong" directory, fujprog.exe
will exit without printing any error or any other message while
it should print help/usage message shown on top of this page.
Possible cause of this problem is that "ftd2xx64.dll" (for win64)
is found copied to System32 directory under name "ftd2xx.dll" (for win32).

Possible solution would be to remove all ftd2xx copies and copy
fujprog.exe and dll to another directory and try again.

## LINUX

Here we have much better success, fujprog is statically linked and
doesn't depend on any other file. Most issues come from user permissions
so fujprog should be either run as root or the user should be given
permissions to access USB and serial device, that's usually done
easily with some udev rule:

    # /etc/udev/rules.d/80-fpga-ulx3s.rules
    # this is for usb-serial tty device
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
      MODE="664", GROUP="dialout"
    # this is for fujprog libusb access
    ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", \
      GROUP="dialout", MODE="666"

## APPLE

Make sure that FTDI driver is not loaded. For example:

```
sudo kextstat | grep -i ftdi
```

Should not return com.FTDI.driver.FTDIUSBSerialDriver when ulx3s is connected to USB port like this:

```
195 0 0xffffff7f85521000 0x7000 0x7000 com.FTDI.driver.FTDIUSBSerialDriver (2.4.4)
```

If does, try to run fujprog as root as it will try to automatically handle kexts. Also, you can temporarily remove them by running following commands:

```
/sbin/kextunload -bundle-id com.FTDI.driver.FTDIUSBSerialDriver
/sbin/kextunload -bundle-id com.apple.driver.AppleUSBFTDI
```

Feel free to report any problems. If you have problems running released binary try to compile your own version.

# Credits

Fujprog is based on ujprog originally authored by Marko Zec.
Contributions from EMARD, gojimmypi and kost.

Copyright (C), 2008-2018 Marko Zec, University of Zagreb

Copyright (c), 2014-2019 EMARD

Copyright (c), 2020 kost

