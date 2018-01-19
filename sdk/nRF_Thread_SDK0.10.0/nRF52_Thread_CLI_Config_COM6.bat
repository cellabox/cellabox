mode COM6 BAUD=115200 PARITY=n DATA=8
echo panid 0xabcd >COM6
echo ifconfig up >COM6
echo thread start >COM6
pause