# AmigoBot and WiFi

# Network configuration

## USB to Serial Adapter Delock

On Windows/MacOS you need to install a driver to use the Serial adapter available [here](https://www.delock.de/produkte/G_61425/merkmale.html?setLanguage=en)

## access settings

ESSID: `AmigoBosch`

Encryption: WPA2 with TKIP (FritzBox: _WPA + WPA2_)

Password: `All those fancy little robots!`

Robot IPs: `10.0.126.1{1,2,3,4,5}`

DHCP: from 10.0.126.20 to 10.0.126.200

Network settings: gateway 10.0.126.1 (router IP), netmask 255.255.255.0

To connect to the device when in WiFi, just access the IP address using a web browser.
Make sure to not use any proxy!
Both password and username are blank.


## Initial settings

ESSID: `Wireless Network`

IP: 10.0.126.1

No password


## All the robots


Name | IP | Mac-Address
---|---:|---:
AchimBot | 10.0.126.11 |    00:20:4A:FA:7E:5C
BarbaraBot | 10.0.126.12 |  00:20:4A:FA:7E:50
CorinnaBot | 10.0.126.13 |  00:20:4A:FA:7E:6F
DominikBot | 10.0.126.14 |  00:20:4A:FA:7E:7C
EmilBot | 10.0.126.15 |     00:20:4A:FA:7E:7B


## Use with RosAria


To work with the robot, use

    rosrun rosaria RosAria _port:=10.0.126.11:8101



# Change network configuration in robot (Lantronix)

Use Null-Modem-Cable from package, attach Serial 1 from Lantronix to
docking station serial port.

Access terminal per

    sudo screen /dev/ttyS0 9600

Hold down `x` while starting AmigoBot, then when some text appears press
_Enter_.




# Full Lantronix default dump

```
*** Lantronix WiBox Device Server ***
MAC address 00204AFA7E5C
Software version V6.8.0.4 (130110)
AES library version 1.8.2.1
Press Enter for Setup Mode


*** basic parameters
Hardware: Ethernet TPI, WLAN 802.11bg
Network mode: Wireless Only
IP addr 10.0.126.11, gateway 10.0.126.1,netmask 255.255.255.0
DNS Server not set
DHCP FQDN option: Disabled

*** Security
SNMP is              enabled
SNMP Community Name: public
Telnet Setup is      enabled
TFTP Download is     disabled
Port 77FEh is        enabled
Web Server is        enabled
Web Setup is         enabled
ECHO is              disabled
Encryption is        disabled
Enhanced Password is disabled

*** Channel 1
Baudrate 9600, I/F Mode 4C, Flow 00
Port 08101
Connect Mode : C0
Send '+++' in Modem Mode disabled
Show IP addr after 'RING' enabled
Auto increment source port disabled
Remote IP Adr: --- none ---, Port 00000
Disconn Mode : 00
Flush   Mode : 00

*** Channel 2
Baudrate 115200, I/F Mode 4C, Flow 00
Port 08102
Connect Mode : C0
Send '+++' in Modem Mode disabled
Show IP addr after 'RING' enabled
Auto increment source port disabled
Remote IP Adr: --- none ---, Port 00000
Disconn Mode : 00
Flush   Mode : 00

*** Expert
TCP Keepalive    : 45s
ARP cache timeout: 600s
CPU performance: Regular
Monitor Mode @ bootup : enabled
HTTP Port Number : 80
MTU Size: 1400
TCP Re-transmission timeout: 500 ms
Alternate MAC: disabled
Ethernet connection type: auto-negotiate

*** WLAN
WLAN: enabled
Topology: Infrastructure
Network name: Wireless Network
Country: US
Security suite: none
TX Data rate: 1 Mbps fixed
Minimum TX Data rate: 1 Mbps
Power management: disabled
Soft AP Roaming: disabled
WLAN Max failed packets: 6
```


# full dump with 'our' settings


```
*** basic parameters
Hardware: Ethernet TPI, WLAN 802.11bg
Network mode: Wireless Only
IP addr 10.0.126.11, gateway 10.0.126.1,netmask 255.255.255.0
DNS Server not set
DHCP FQDN option: Disabled

*** Security
SNMP is              enabled
SNMP Community Name: public
Telnet Setup is      enabled
TFTP Download is     disabled
Port 77FEh is        enabled
Web Server is        enabled
Web Setup is         enabled
ECHO is              disabled
Encryption is        disabled
Enhanced Password is disabled

*** Channel 1
Baudrate 9600, I/F Mode 4C, Flow 00
Port 08101
Connect Mode : C0
Send '+++' in Modem Mode disabled
Show IP addr after 'RING' enabled
Auto increment source port disabled
Remote IP Adr: --- none ---, Port 00000
Disconn Mode : 00
Flush   Mode : 00

*** Channel 2
Baudrate 115200, I/F Mode 4C, Flow 00
Port 08102
Connect Mode : C0
Send '+++' in Modem Mode disabled
Show IP addr after 'RING' enabled
Auto increment source port disabled
Remote IP Adr: --- none ---, Port 00000
Disconn Mode : 00
Flush   Mode : 00

*** Expert
TCP Keepalive    : 45s
ARP cache timeout: 600s
CPU performance: Regular
Monitor Mode @ bootup : enabled
HTTP Port Number : 80
MTU Size: 1400
TCP Re-transmission timeout: 500 ms
Alternate MAC: disabled
Ethernet connection type: auto-negotiate

*** WLAN
WLAN: enabled
Topology: Infrastructure
Network name: AmigoBosch
Country: US
Security suite: WPA2/802.11i
Authentication: PSK
Encryption: TKIP
TX Data rate: 1 Mbps fixed
Minimum TX Data rate: 1 Mbps
Power management: disabled
Soft AP Roaming: disabled
WLAN Max failed packets: 6
```
