
Figure out ip address
```
cat /var/lib/misc/dnsmasq.leases
```

Test this firmware with
```
socat -,rawer,echo,escape=0x03 TCP:10.42.0.54:23
```
