# MarantecQ7900TX
Photon Particle project for sending garage door RF remote key codes. Uses PWM and corresponding interrupts (timer)

This code sends a 24bit pulse train to a Marantec Q7900 Garage Door Opener using a Photon Particle and a 315MHz transmitter board bought on Amazon.

Obatin the "secret" 24bit code using my MarantecQ7900RX program. Modify this TX code with the captured key and then control your Q7900 opener via the internet. 

(My ultimate goal is to add Particle support to my Homebridge server and use Home-Kit via my iPhone.)
