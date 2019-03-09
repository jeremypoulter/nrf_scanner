# nRF Scanner

## Overview

[![Build Status](https://travis-ci.org/jeremypoulter/nrf_scanner.svg?branch=master)](https://travis-ci.org/jeremypoulter/nrf_scanner)

This is a 2.4GHz RSSI scanner based on the [nRF Connect](https://github.com/NordicSemiconductor/pc-nrfconnect-core) example [RSSI Viewer](https://github.com/NordicSemiconductor/pc-nrfconnect-rssi), but designed to use an [
Adafruit Feather nRF52 Bluefruit LE - nRF52832](https://www.adafruit.com/product/3406) with a [TFT FeatherWing](https://www.adafruit.com/product/3315) to create a portable, self contained scanner.

![Photo](doc/example.jpg)

## Building

### Visual Studio Code

The eaiest way to build and modify this project is with [Visual Studio Code](https://code.visualstudio.com/).

Once VSCode is installed, clone this repository and open the newly cloned folde in VSCode (File->Open Folder...).

You will need to install the [PlatformIO IDE](https://platformio.org/) extension from the Extensions side bar. This should show up as a recommended extension.

### Command line

```shell
git clone https://github.com/jeremypoulter/nrf_scanner.git
cd nrf_scanner
platformio run
```
