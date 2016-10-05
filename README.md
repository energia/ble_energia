ble_energia
===========

This repository hosts the active development for TI's BLE Energia solution. ble_energia adds Energia support to the CC2650 in network processor configuration, using the MSP432 as the application processor.

**Note: This repo hosts the bleeding edge versions of ble_energia, stable releases are bundled within the Energia IDE's board manager package for MSP432**

Required Hardware and Software
==============================

* The software in this repo
* [Energia 18 (1.6.10E18)](http://energia.nu/download/)
* [MSP432P401R LaunchPad](http://www.ti.com/tool/msp-exp432p401r)
* CC2650 [LaunchPad](http://www.ti.com/tool/launchxl-cc2650) or [BoosterPack](http://www.ti.com/tool/boostxl-cc2650ma) running the Simple Network Processor Image
 - The sketches currently build with power savings. You can get SNP images from [this link](http://software-dl.ti.com/dsps/forms/self_cert_export.html?prod_no=ble_2_02_simple_np_setup.exe&ref_url=http://software-dl.ti.com/lprf/BLE-Simple-Network-Processor-Hex-Files).
 - For CC2650MOD BoosterPack you will want to use `simple_np_cc2650bp_uart_pm_xsbl.hex`
 - For CC2650_LAUNCHXL you will want to use `simple_np_cc2650lp_uart_pm_xsbl.hex`

Steps to setup
==============

1. Clone or download this repository
2. Install MSP432 Board files

  * Tools -> Board -> Board Manager
3. Symlink to this repo
  * Navigate to energia-1.6.10E18B6-\<platform\>/energia-1.6.10E18/libraries
    * Note that platform above refers to your OS (i.e. windows)

   **Windows**
     * In `git bash` type `ln --symbolic "<PATH_TO_THIS_REPO>\BLE" BLE`
     * If using `cmd` use `mklink /J BLE "<PATH_TO_THIS_REPO>\BLE"`


   **Linux/Mac**
     * In `terminal` type `ln -s "<PATH_TO_THIS_REPO>\BLE" BLE`
