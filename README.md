Tangerine nRF51 beacon firmware
==================

Overview

## Description
nRF51 beacon firmware for Tangerine.

## Features
This firmware supports following protocols.
* Tangerine iBeacon
* Tangerine Beacon Management Service
* 2 EddyStone-URLs
* EddyStone-UUID
* EddyStone-TLM
* LINE Beacon

## Support Devices
* RapiNavi Air 2
* nRF51-DK (nRF6824, PC10028)

## Requirement
* Toolchain: gcc-arm-none-eabi-4_9-2015q3  
  [linkref] https://launchpad.net/gcc-arm-embedded "GNU ARM Embedded Toocahaind"

* Makefile template   
  git clone https://github.com/hlnd/nrf51-pure-gcc-setup.git

### Softdevie8
* SoftDevice: s110_nrf51822_8.0.0
  [linkref] http://developer.nordicsemi.com/nRF5_SDK/

* Nordic SDK: nRF51_SDK_8.1.0_b6ed55f
  [linkref] http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822

## Versions and Tags(master branch)

| version | tag         | branch           |  new support features           |
|---------|:------------|:-----------------|:-------------------------------:|
| v40992  | v40992      | master           | original                        |
| v41008  | v41008      | master           | eddystone-url                   |
| v41009  | v41009      | master           | new settings                    |
| v41024  | v41024      | master           | eddystone-uuid eddystone-tlm    |
| v41040  | v41040      | master           | refactoring                     |
| v41056  |             | master           | LINE beacon                     |
| v41072  |             | feature/tgsecure | Tagnerine Secure iBeacon        |

## Third-party crypto libraries
This firmware uses algorithm from the following third-party cryptographic libraries.

| Library | Algorithm | License |
| -------:|----------:|--------:|
| [RFC6234](https://github.com/massar/rfc6234) | HMAC-SHA256 | [Simplified BSD License](https://en.wikipedia.org/wiki/BSD_licenses#2-clause_license_.28.22Simplified_BSD_License.22_or_.22FreeBSD_License.22.29) |

## Build
$ cd armgcc 
$ make clean 

## Flash to device: using JLink
$ cd armgcc 
$ make erase-all 
$ make flash-softdevice  
$ make flash  

## How to Debug
To degug with gdb, you need two terminals.　

On 1st terminal:  
$ cd armgcc  
$ ./gdb-server.sh  

On 2nd termial:  
$ cd armgcc  
$ ./gdb-debug.sh  
(gdb) target remote local:2331  
