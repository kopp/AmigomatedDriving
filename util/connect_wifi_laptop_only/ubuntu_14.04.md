# Direct connection from your laptop to a robot

This assumes, that you use the *NetworkManager*.
It was tested with network controller (`lspci`)

     Network controller: Intel Corporation Wireless 7260 (rev 6b)

Add the file `/etc/NetworkManager/system-connections/LocalAmigoBot` with
content

    [connection]
    id=LocalAmigoBot
    uuid=8897b2aa-fbbb-4da2-a2a0-1b4684797700
    type=802-11-wireless
    autoconnect=false
    timestamp=1487938196

    [802-11-wireless]
    ssid=AmigoBosch
    mode=ap
    mac-address=A4:C4:94:4A:DA:85
    seen-bssids=00:00:00:00:00:00;
    security=802-11-wireless-security

    [802-11-wireless-security]
    key-mgmt=wpa-psk
    psk=All those fancy little robots!

    [ipv4]
    method=manual
    address1=10.0.126.1/24,10.0.126.1
    may-fail=false

    [ipv6]
    method=ignore

This allows you to create a wifi access point, see
[here](http://askubuntu.com/questions/318973/how-do-i-create-a-wifi-hotspot-sharing-wireless-internet-connection-single-adap)
for more info.

You will need to set the `mac-address` to the one of your wifi module
(typicallly `wlan0`; use `ifconfig wlan0` to see the mac address, called
`HWaddr` here).  For good measure, create a new `uuid`
[here](https://www.uuidgenerator.net/).  Note, that the file name and the name
in `id` must match.  Note also, that whenever you change the setting with
`nm-connection-editor`, you will have to change the file manually so that it
contains `mode=ap`.


To enable this connection

- click on the network manager applett (`nm-applet`),
- then on *Connect to hidden WiFi network*
- and here select the connection *LocalAmigoBot* (or whatever you called it).

The robot is configured in such a way to connect to the wifi and acquire a
fixed ip which is printed out on its underside.  You should then be able to run
e.g.

    rosrun rosaria RosAria _port:=10.0.126.14:8101

to connect to it.


# Allow other people to use the robot

If you want to allow other people to connect to the robot as well, using your
laptop as "router", install `isc-dhcp-server`, add

    subnet 10.0.126.0 netmask 255.255.255.0 {
            range 10.0.126.20 10.0.126.200;
            interface wlan0;
            option subnet-mask 255.255.255.0;
    }

to `/etc/dhcp/dhcpd.conf` and start the dhcpd service via

    sudo service isc-dhcp-server start

Other people should now be able to log in to wifi `AmigoBosch`, connect to the
robots or to your machine.
