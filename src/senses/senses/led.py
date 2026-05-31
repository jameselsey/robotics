from gpiozero import PWMLED
from time import sleep

led = PWMLED(19)  # Use GPIO18 if possible

try:
    while True:
        led.value = 0.5  # 50% brightness
        sleep(1)
        led.value = 1.0  # Full brightness
        sleep(1)
except KeyboardInterrupt:
    led.off()