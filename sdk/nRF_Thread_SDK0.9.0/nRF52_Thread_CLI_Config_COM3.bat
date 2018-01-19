mode COM3 BAUD=115200 PARITY=n DATA=8
echo panid 0xabcd >COM3
echo ifconfig up >COM3
echo thread start >COM3
pause