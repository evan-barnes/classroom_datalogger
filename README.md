# backpacking_datalogger
Multi-sensor datalogger for citizen science and classroom use.

The ultimate goal of this project is to create a versatile, inexpensive, multi-sensor data logger as an open source and open hardware project,
aimed at individuals who want to do citizen science, or at schools who want to use a system like this in the classroom. While there already are
dataloggers available for classroom use, they are expensive and rely on proprietary software. I want to create an option for interested individuals and
teachers that will cost less than about $100 per unit, and which will use commonly available or open source software to analyze the data that is collected. 

The design is still in the extreme early stages. I'm still figuring out open source licensing, but to declare my intentions now, the hardware design will
probably be licensed under the CERN Open Hardware License version 2 (or possibly Creative Commons CC BY-SA 4.0), and the firmware and any extra analysis 
software I write will probably be licensed under the MIT open source license. My current early prototype is built primarily on Adafruit components, so I
may need to follow certain licensing because of that.

In broad terms, I want to ensure that the software and hardware designs are open and available for anyone to use, make, and modify in any way that they
see fit, as long as they share their modifications under a similar license and provide proper attribution to the original project. In the long term, 
I intend to design custom PCBs for the primary datalogger and any external sensors that will attach to it, and I'll make the design files available for anyone
to have manufactured. I also intend to have a small stock of them manufactured and assembled so that I can sell them ready-made to people who don't want
to make them themselves. Basically, in my own small way, I want this to work like an Adafruit or Arduino project, and the licensing will be structured
as such.

In the long term, I envision this project as a small SD card datalogger with a screen, a few buttons for interaction, and a few built-in sensors, all 
on a custom PCB. The sensors built into the datalogger will include things like temperature, barometric pressure, humidity, UV index, 9-axis IMU (accelerometer, gyroscope, and magnetometer), a particulate matter air quality sensor, air quality sensors that detect things like CO and VOCs, and GPS for geospatial tagging
of logged data. There will also be expansion ports for other sensors, like water quality sensors that detect total dissolved solids, water temperature, and turbidity.
I plan to have the sensors connect to the datalogger with USB-C connections, and am working on having them identify themselves to the datalogger so that no
additional setup or reprogramming will be required by the user. 

This is all a long way off still. The current design has all those sensors, but they're Adafruit expansion boards connected to an Adafruit Feather Sense MCU,
all built on perf board and enclosed in box made of hot glued corrugated plastic. I wanted to take it on a trip and needed a quick enclosure, and it worked
out well enough for testing! Next steps will involve a pretty heavy redesign of both the hardware and software. I'm leaning toward switching the MCU for 
either an ESP32, or as I've come to favor lately, a Teensy. The NRF52840 in the Feather Sense is great, but it's been struggling to keep up with all the 
timing requirements for things like GPS, logging data to the SD card, and writing to the screen. I'm sure that's mostly my amateur code, but I think leaving
some future-proofed room for expansion in terms of MCU power is a good idea, especially since I also want to add on-board processing of IMU data to 
yield quaternion orientation output. 
