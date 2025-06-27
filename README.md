# mirte-demo-ensurance
This repo contains extra configurations for the 2025 summer school.

To setup the MIRTE connect it to the internet via the provided instructions.
Then execute the `install_mirte.sh` script on the robot by excuting the following (when logged into the robot)
`curl --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/SuperJappie08/mirte-demo-ensurance/refs/heads/development-detection/install_mirte.sh | bash`

====
While running docker you might run into the issue where the MIRTE cannot connect to the internet over the shared ethernet connection.
To resolve this find the ethnernet and wifi (wlan) interface by running `ip a` on your machine.
Then execute the following, to fix it temporarly (untill reboot):
```
ethernet=ETHERNET_ADAPTER_TO_MIRTE
wifi=OTHER_INTERNET_CONNECTION
sudo iptables -A FORWARD -i $ethernet -o $wifi -j ACCEPT
sudo iptables -A FORWARD -i $wifi -o $ethernet -m state --state ESTABLISHED,RELATED -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o $wifi -j MASQUERADE
```