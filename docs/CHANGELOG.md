# Changelog

All notable changes to the FarmPulse Firmware will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v0.1.0] - 2026-06-14
### Added
- Initial MQTT component inetegration in firmware for makeshift zero-PCB hardware completed.
- Setup of MQTT frame format. And also communication established between Gateway and Cloud/Web interface.
- LoRa transreception Frame formating and communication completed.
- Automated version embedding via ESP-IDF and Git tags.

### Known Issues
- Internet access is stricitly through the WiFi access.
- OTA update functionality is currently bypassed/pending.