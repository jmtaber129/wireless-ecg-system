# ECG System

Code for an embedded ECG system.

This repository includes code for group 5 of Texas Tech's ECE 3332-301 project lab course.

A web application for viewing waveforms is located at http://ecg-web-viewer.appspot.com/

Phase 1 (jmtaber129/wired-ecg-system) includes code for one MSP430G2553, and is designed as a fully wired system, with an amplified ECG signal being input to an analog pin, and a UART connection to a PC for transmitting serialized analog readings and receiving commands.

Phase 2 (this repo) includes code for two MSP432P401R's, and is designed as a wireless system, in which data is sent between two microcontrollers and a PC via Bluetooth.
