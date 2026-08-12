# ESPHome-SAME-Decoder
Process live National Weather Service radio data for real-time, Internet-independent alert data in Home Assistant.


VERY MUCH A WORK-IN-PROGRESS!


## Hardware
THIS IS INCOMPLETE AS I AM STILL WORKING ON THE HARDWARE SIDE OF THIS.
* ESP32 Audio Dev Kit such as [this](https://www.aliexpress.us/item/3256804762753719.html)

## Installation
To install: use [this file](https://github.com/infinitytec/ESPHome-SAME-Decoder/blob/main/config/noaa-same-decoder.yaml) in ESPHome Device Builder as your device's config.


### Generative AI Disclaimer:
I am using Generative AI tools to develop this. I am using GitHub to help me track changes to prevent introducing new bugs and to speed up review between versions. Contributions using Generative AI are welcome.






### To-Do:
[] Make the decoding more robust
[] Decoding seems to really like to listen in groups of threes (understandable) but if it misses one and another alert comes it seems to not be happy. So it should time out (less than 30 seconds?) and send what it's got.
[] SNR for NOAA signals would be cool and useful.
[] Refactor decoder to work better with off-frequency tuning (not sure how necessary this is).
