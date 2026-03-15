# WiFi-Shuriken

WiFi-Shuriken is a distributed WiFi scanning and logging system built around two roles, controller and scanner. An RP2350-based controller manages the overall system, while one or more ESP32-C5 scanner nodes perform channel-specific WiFi scans and report results back to the controller.

The controller handles functions such as USB serial, GNSS, SD logging, and scan scheduling. It talks to the scanner nodes over SPI, with optional shift-register based chip select fanout for multiple scanners. The controller firmware splits work across cores so scanner scheduling and transport can run independently from logging and GNSS processing.

Each ESP32-C5 scanner runs as an SPI slave and exposes a small command protocol for identity, scan requests, result counts, and result retrieval. Scan results are deduplicated and buffered on the scanner, then deduplicated again at the controller before being written to WiGLE CSV format with timestamp and position data from the GNSS subsystem.
