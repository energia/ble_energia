ble_energia setup
=================

This page describes how to get setup with Energia the TI BLE library. Information on this guide is for internal use only.
End users will get a working out of the box install from the Energia installer.


Required Hardware and Software
==============================

* The software in this repo
* Energia 15 beta
* MSP432P401R LaunchPad
* CC2650 LaunchPad or boosterpack running the Simple Network Processor Image
 - The sketches currently build with power savings. You can get SNP images from [this link](http://software-dl.ti.com/dsps/forms/self_cert_export.html?prod_no=ble_2_02_simple_np_setup.exe&ref_url=http://software-dl.ti.com/lprf/BLE-Simple-Network-Processor-Hex-Files).
 - For CC2650MOD BoosterPack you will want to use `simple_np_cc2650bp_uart_pm_xsbl.hex`
 - For CC2650_LAUNCHXL you will want to use `simple_np_cc2650lp_uart_pm_xsbl.hex`

Steps to setup
==============


1. Setup Energia 18 beta, steps below are from Robert Wessels

# Energia 18 instructions:

All previous beta’s should be deleted and all files associated with it as well since the package_index.json versioning numbering changed. Please follow the instructions below for your operating system.
The redistributable JRE has been included in Energia itself so you should not see any dependencies on external JRE’s.

It default comes with MSP430 boards. Install MSP432, TivaC and CC3200 via "Tools->Boards->Board Manager”
Make sure that if you are within the TI network, that you set the proxy via:

File->Preferences of Energia->Preferences if you are on Mac. Then click the Network tab and set the proxy to “Manual proxy configuration”:
Host name: wwwgate.ti.com
Port number: 80
Leave Username and Password empty.

## Linux64:

1. Before you download, make sure that you delete any previous beta version.
2. Open a command window and then:
            - rm -r ~/.energia15/packages/energia/
            - rm -r ~/.energia15/staging/packages
            - rm ~/.energia15/library_index.json
            - rm ~/.energia15/package_index.json
            - rm ~/.energia15/package_index.json.sig
3. Now download the IDE from: https://s3.amazonaws.com/energiaUS/distributions/energia-1.6.10E18B6-linux64.tar.xz
4. You should now be able to unpack with tar -xf energia-1.6.10E18B6-linux64.tar.xz and then run energia in the containing folder.

## Mac OS X:

1. Before you download, make sure that you delete the previous version beta
2. Open a command window and then:
            - rm -r ~/Library/Energia15/packages/energia/
            - rm -r ~/Library/Energia15/staging/packages
            - rm -r ~/Library/Energia15/library_index.json
            - rm -r ~/Library/Energia15/package_index.json
            - rm -r ~/Library/Energia15/package_index.json.sig
3. Now download the IDE from: https://s3.amazonaws.com/energiaUS/distributions/energia-1.6.10E18B6-macosx-signed.zip
4. You should now be able to unzip and run it.

## Windows:

1. Before you download, make sure that you delete the previous version beta (if you had not installed a previous beta then skip to step 5).
2. Open a file explorer window and type in %LOCALAPPDATA%\Energia15
3. Delete the folder packages
4. Delete the folder staging
5. Delete the files package\_index.json, package\_index.json.gz and library\_index.json
5. Now download the IDE from: http://s3.amazonaws.com/energiaUS/distributions/energia-1.6.10E18B6-windows.zip
6. You should now be able to unzip and run it.


# Install MSP432 Board files

1. Tools -> Board -> Board Manager
 - Note that doing this from a TI network will require the proxy settings from above



# Override MSP432 platform.txt

This is required to throw additional #defines required by the BLE library.

1. Copy the platform.txt from this folder to `%LOCALAPPDATA%\Energia15\packages\energia\hardware\msp432\1.3.3`.
2. Override the current platform.txt

**Note that if you would like to disable POWER_SAVINGS (use two wire interface, no handshaking), remove this symbol from platform.txt**

# Symlink to this repo

1. Navigate to energia-1.6.10E18B6-windows/arduino-1.6.10E18B6/libraries
2. In `git bash` type `ln --symbolic "<PATH_TO_THIS_REPO>\BLE"`
3. If using `cmd` use mklink -J



