## grblHAL my_plugin templates and utilities

To add a plugin to a build copy _my_plugin.c_ to the folder where _driver.c_ is found.  
Do NOT copy _CMakeLists.txt_ as this is used exclusively by the [Web Builder](http://svn.io-engineering.com:8080/) and may interfere with local builds.

__Note:__ To add a plugin to the firmware for RP2040 and ESP32 builds enable the _AddMyPlugin_ option in _CMakeLists.txt_.

---
2023-08-09
