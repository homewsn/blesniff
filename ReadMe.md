[![Release Status](https://img.shields.io/github/release/homewsn/blesniff.svg)](https://github.com/homewsn/blesniff/releases)

### Blesniff

Blesniff is a Bluetooth Low Energy sniffer implemented as firmware for nRF5340 hardware modules. It uses a virtual USB serial port to communicate with [Bsniffhub](https://github.com/homewsn/bsniffhub) for capturing, decrypting, and displaying wireless traffic in Wireshark.<br>
Blesniff provides the following features:
* Support for connection requests on both primary and secondary advertising channels
* Support for the Connected Isochronous Streams (CIS) and Groups (CIG)
* Support for extended non-periodic and periodic advertising (including PAwR)
* Support for Broadcast Isochronous Streams (BIS) and Groups (BIG), including both interleaved and sequential BIS arrangements
* Support for packet filtering on primary advertising channels by MAC address and minimum RSSI
* Support for scanning all three primary advertising channels to increase the likelihood of capturing the packets. The sequence and number of channels to be scanned are configurable
* Traffic decryption, if the LTK is already known or provided

#### Building
Since the nRF5340 is a dual-core SOC, it is necessary to build and flash firmware for both processor cores - app_core and net_core. The firmware source code is implemented as a pure bare-metal application, so only a toolchain is required for compilation (GCC or IAR). The package includes both Makefile and IAR project file for each core.
Two firmware variants are available for the nRF5340 app_core:
* One with UART controller support
* One with USB device (USBD) controller support

If you choose the UART version, you will need an external USB-to-UART adapter. Using the USBD version requires an external USB connector. For example, both the E83-2G4M03S-TB and NRF5340-DK meet these requirements. Flashing the E83-2G4M03S-TB requires an external programmer, while the NRF5340-DK has a built-in one.<br>
Ready-to-use firmware files are available on the [Releases](https://github.com/homewsn/blesniff/releases) page.

#### Usage
To use the sniffer, you'll need [Bsniffhub](https://github.com/homewsn/bsniffhub). Use the console version with the `-s B` option, or select `Blesniff` in the graphical version of [Bsniffhub](https://github.com/homewsn/bsniffhub). Additional options available:
```
  -b <baudrate>      Serial port baudrate
                     (def: 1000000)
  -c <channel(s)>    Primary advertising channel(s) to listen on: 37, 38, 39
                     (def: 37,38,39)
  -R <RSSI>          Filter sniffer packets by minimum RSSI
  -m <MAC>           Filter sniffer packets by advertiser MAC
  -f <MODE>          Blesniff follow mode:
                     'conn' - connection
                     'pa' - periodic advertising
                     'cis' - connected isochronous stream ('conn' is also required)
                     'bis' - broadcast isochronous stream ('pa' is also required)
                     (def: conn,pa,cis,bis)
```

#### Examples (Linux)
Run Wireshark, capture packets from `Blesniff` on `/dev/ttyUSB2` port and feed the captured packets with the  `LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR` packet header to `Wireshark`, follow the first event that happens (connection, periodic advertising, CIS, and BIS packets):
```
$ ./bsniffhub -s B -p /dev/ttyUSB2
```
Run Wireshark, capture packets from `Blesniff` on `/dev/ttyUSB2` port and feed the captured packets with the  `LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR` packet header to `Wireshark`, follow only connections `conn` (ignore periodic advertising, CIS, and BIS packets), ignore all legacy advertising packets except packets with random MAC address `59:5e:58:6f:18:7d`:
```
$ ./bsniffhub -s B -p /dev/ttyUSB2 -f conn -m 59:5e:58:6f:18:7dr
```
Run Wireshark, capture packets from `Blesniff` on `/dev/ttyUSB2` port and feed the captured packets with the  `LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR` packet header to `Wireshark`, follow only connections `conn` and CIS packets `cis` (ignore periodic advertising and BIS packets), use LTK `6ab0580e966e7b61f4470dfb696b3799` for decryption:
```
$ ./bsniffhub -s B -p /dev/ttyUSB2 -f conn,cis -L 6ab0580e966e7b61f4470dfb696b3799
```
Run Wireshark, capture packets from `Blesniff` on `/dev/ttyUSB2` port and feed the captured packets with the  `LINKTYPE_NORDIC_BLE` packet header to `Wireshark`, ignore advertising packets on the primary advertising channels with RSSI less than `-70`, follow only periodic advertising `pa` and BIS packets `bis` (ignore connections and CIS packets):
```
$ ./bsniffhub -s B -p /dev/ttyUSB2 -l 272 -R -70 -f pa,bis
```

#### Examples (Windows)
See [Examples (Linux)](#examples-linux), but you can use the additional optional argument `-W` if Wireshark was installed in a path other than `C:\Program Files\Wireshark\Wireshark.exe`:
```
> bsniffhub -s B -p COM40 -W D:\\Wireshark\\Wireshark.exe
```

#### Examples of PCAP files
Examples of sniffing on various BLE sessions can be found in the examples directory. Applications from nRF Connect SDK 3.0.0 and samples from the corresponding Zephyr SDK were used for testing.

nrf5340_audio, unicast_client (headset) + nrf5340_audio, unicast_server (gateway):
```
> bsniffhub -sB -pCOM42 -R-70 -fconn -wexamples/nrf5340_audio_unicast_client_with_unicast_server_1.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -46 dBm 56:0d:2c:d5:68:8c random detected.
Do you want to follow only this device? No
Connection created.
Channel selection algorithm #2 detected.
Connection established.
Encryption request detected, but LTK unknown.
Do you have the Long Term Key (LTK)? No
Encryption start detected. LTK used.
```
```
> bsniffhub -sB -pCOM42 -R-70 -fconn -L6ab0580e966e7b61f4470dfb696b3799 -wexamples/nrf5340_audio_unicast_client_with_unicast_server_2.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -43 dBm 60:8f:69:59:be:72 random detected.
Do you want to follow only this device? No
Connection created.
Channel selection algorithm #2 detected.
Connection established.
Encryption start detected. LTK used.
```
```
> bsniffhub -sB -pCOM42 -R-70 -fconn,cis -L6ab0580e966e7b61f4470dfb696b3799 -wexamples/nrf5340_audio_unicast_client_with_unicast_server_3.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -45 dBm 48:24:38:45:82:dc random detected.
Do you want to follow only this device? No
Connection created.
Channel selection algorithm #2 detected.
Connection established.
Encryption start detected. LTK used.
```
nrf5340_audio, broadcast_source:
```
> bsniffhub -sB -pCOM42 -R-70 -fpa -wexamples/nrf5340_audio_broadcast_source_1.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -48 dBm e0:fb:ce:9f:f1:ed random detected.
Do you want to follow only this device? No
```
```
> bsniffhub -sB -pCOM42 -R-70 -fpa,bis -wexamples/nrf5340_audio_broadcast_source_2.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -44 dBm e0:fb:ce:9f:f1:ed random detected.
Do you want to follow only this device? No
```
periodic_adv_rsp + periodic_sync_rsp:
```
> bsniffhub -sB -pCOM42 -R-70 -fconn -wexamples/periodic_adv_rsp_with_periodic_sync_rsp_1.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -43 dBm 11:4e:d3:c7:5f:83 random detected.
Do you want to follow only this device? No
BLE device -41 dBm eb:e4:22:ab:31:94 random detected.
Do you want to follow only this device? No
Connection created.
Channel selection algorithm #2 detected.
Connection established.
Connection terminated.
Connection created.
Channel selection algorithm #2 detected.
Connection established.
Connection terminated.
```
```
> bsniffhub -sB -pCOM42 -R-70 -fpa -wexamples/periodic_adv_rsp_with_periodic_sync_rsp_2.pcap
Connection to Wireshark established.
Blesniff hardware detected and started.
BLE device -44 dBm 15:95:f0:c0:c9:18 random detected.
Do you want to follow only this device? No
```
