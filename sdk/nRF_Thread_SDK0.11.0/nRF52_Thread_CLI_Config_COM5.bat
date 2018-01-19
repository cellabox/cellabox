mode COM5 BAUD=115200 PARITY=n DATA=8
echo panid 0xabcd >COM5
echo ifconfig up >COM5
echo thread start >COM5
pause