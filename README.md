# ESPHome-SAME-Decoder

Process live National Weather Service radio data for real-time, Internet-independent alert data in Home Assistant.

THIS IS STILL A WORK-IN-PROGRESS.

## Hardware

THIS IS INCOMPLETE AS I AM STILL WORKING ON THE HARDWARE SIDE OF THIS.

* ESP32 Audio Dev Kit such as [this](https://www.aliexpress.us/item/3256804762753719.html). This is the board this project is developed for. Other boards will likely work but also need configuration changes. The important part is audio line-in.
* A weather radio such as [this](https://www.aliexpress.us/item/3256806595493109.html?). Note that the radio needs to be always on (no battery saving timeout), tunes to the Weather Band or a local weather station that broadcasts SAME alerts, and offers an audio output.
* A battery replacer for the radio to enable the radio to always be on without destroying batteries constantly. I currently have no recommendation for this as the one I got creates a lot of RF noise.
* A power supply for the ESP32 board. A battery may also be a good idea.
* An audio cable to connect the radio to the board.
* A nice box to put things in.

## Installation

Tune the radio to your [local NOAA station](https://www.weather.gov/nwr/station_search). Also identify your SAME (FIPS) Code.

> Note that most counties in the US send alerts county-wide. Some counties have more granular reporting. Make sure you check for your specific area.

To install: use [this file](https://github.com/infinitytec/ESPHome-SAME-Decoder/blob/main/config/noaa-same-decoder.yaml) in ESPHome Device Builder as your device's config.

WIP

## Important Disclaimer

> NOTE: Do not rely on this for safety. This is designed as a supplemental and informational project. Reliance of alerts from this is at your own risk.
> 
> Just a sampling of things that can go wrong:
> 
> * Bugs and/or insufficient software may cause alerts to be missed.
> * Home Assistant may be offline.
> * Improper receiver placement may cause alerts to be missed.
> * NOAA Weather Radio may be unavailable.
> * Use of Generative AI in the development of this project may result in bugs, edge cases, and unpredictable behavior.
> 
> Since weather alerts are often sent in life-or-death situations make sure you have other ways to receive critical alerts. Some of those are:
> 
> * Commercially-available weather radios (such as from Midland).
> * Wireless Emergency Alerts on mobile phones.
> * Local TV and radio stations.
> * Social media.
> 
> Stay weather aware and have proper preparedness in place.

If you have the ability to validate that this project is reliable enough to be used as a safety device, please let me know. Could be interesting!

### Generative AI Disclaimer:

I am using Generative AI tools to develop this. I am using GitHub to help me track changes to prevent introducing new bugs and to speed up review between versions. Contributions using Generative AI are welcome, but disclosure is required.

## To-Do

- [ ] Validate the decoder with actual NOAA Weather Radio broadcast alerts.
- [ ] Build out integration within Home Assistant to properly signal alerts, select desired alerts, configure FIPS codes, etc.
- [ ] Extended goals: on-device alerting, audio passthrough to speakers, display alerts on device, fully stand-alone mode, including configuration.

Code is licensed under the permissive MIT License.

