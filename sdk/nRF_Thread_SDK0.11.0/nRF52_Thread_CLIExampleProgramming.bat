cd examples\thread\experimental\cli\uart\hex
pause
nrfjprog -f NRF52 --log --chiperase --program nrf52840_xxaa.hex --reset
pause