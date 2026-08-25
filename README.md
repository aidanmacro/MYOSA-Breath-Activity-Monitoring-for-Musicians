## BreathSense - MYOSA Musical Breath Activity

<p align="center">
  <img src="mouthpiece_completed.jpeg" width="400"><br/>
  <i>Breathsense: a CO2 Detecting mouthpiece using MYOSA board and sensors.</i>
</p>

## Acknowledgements

- Dr. Vincenzo Pusino
- Ben Allen
- University of Glasgow
- University of Glasgow Technical Staff

## Overview

Musicians rely heavily on breath control to shape dynamics, phrasing, tone and endurance, yet there are few accessible ways to measure breathing behaviour objectively during real performance. This project develops a real-time breath monitoring system designed specifically for musicians, using exhaled CO₂, pressure and temperature data to provide direct insight into how breath is being used while playing. The system is built around the MYOSA development board, providing an IoT-capable platform for sensor integration, wireless communication and real-time data logging. A custom Non-Dispersive Infrared (NDIR) CO₂ sensing system detects exhaled breath, while pressure and temperature measurements provide complementary information about breath activity and environmental variation. By combining these sensing methods with wireless data transmission and live visualisation, the project aims to provide musicians, teachers and researchers with a practical tool for analysing breath strength, timing and consistency during practice and performance.

#### Key Features

  - Custom NDIR CO₂ Sensor to allow most direct measurement. 
  - Pressure sensor for additional cross validation data and enhanced insights. 
  - Built on the MYOSA MCU platform, small enough to clip to a belt or pocket, streaming data live
  - On device OLED display for seeing quick analytics 
  - Python based WiFi/IoT system enabling cloud computing for in depth real time signal processing.


## Demo / Examples

### Images

<p align="center">
  <img src="myosa_breakout.jpg" width="400"><br/>
  <i>Closeup of MYOSA breakout board.</i>
</p>

<p align="center">
  <img src="mouthpiece_on_trombone.jpeg" width="400"><br/>
  <i>Mouthpiece attached to real instrument.</i>
</p>

### Videos

<video controls width="100%">
  <source src="./breathsense_promovid.mp4" type="video/mp4">
</video>


## Features (Detailed)


## 1. NDIR CO₂ Sensing

The primary breath measurement is provided by a custom Non-Dispersive Infrared (NDIR) sensing system. CO₂ absorbs infrared radiation at specific wavelengths, so the concentration of CO₂ in exhaled breath can be inferred from the reduction in transmitted infrared light.

A mid-wave infrared (MWIR) LED is used as the optical source. The LED is pulsed rather than operated continuously, reducing average power consumption while also allowing the system to distinguish the desired optical signal from ambient infrared radiation and other sources of interference.

The emitted infrared light passes through the mouthpiece and the musician's exhaled breath before reaching a photodiode. As the concentration of CO₂ in the airflow changes, the amount of infrared radiation reaching the photodiode changes accordingly. The photodiode output is converted into a measurable voltage using an analogue front end before being sampled by the MYOSA board.

The system uses synchronous detection to improve the measurement. The photodiode is sampled in synchronisation with the LED pulses, allowing the signal measured when the LED is active to be compared with the background signal when it is inactive. This helps reject ambient infrared light and other sources of background variation.

The resulting CO₂ signal is used as a proxy for breath activity. Rather than attempting to measure the musician's breathing volume directly, the system provides a real-time indication of changes in exhaled CO₂ as the musician plays.

## 2. Pressure and Temperature Sensing

The optical CO₂ measurement is complemented by pressure and temperature measurements from a BMP180 sensor connected to the MYOSA board over I²C.

Pressure provides an independent measurement of activity within the mouthpiece. Changes in mouthpiece pressure can therefore be compared with the CO₂ signal to help distinguish genuine breath activity from changes or noise in the optical measurement.

Temperature is also recorded because the operating environment of the sensor can change during use. Monitoring temperature provides additional information for accounting for environmental variation and thermal drift in the measurements.

Combining the three measurements gives a more informative representation of breath activity than relying on the CO₂ channel alone.

## 3. Real-Time Breath Monitoring

The sensor data is processed into a real-time representation of the musician's breath activity. Changes in the measured CO₂ concentration can be visualised as a capnogram, allowing the user to observe the timing and relative strength of their breath while playing.

The pressure signal provides an additional reference alongside the CO₂ measurement, while temperature data can be used to identify environmental changes and compensate for drift.

This allows breath activity to be observed during actual playing rather than requiring the musician to perform a separate breathing test.

## 4. MYOSA Integration and Wireless Data

The sensing hardware is integrated with the MYOSA development board, which acts as the central controller for the system. The board controls the pulsed LED, reads the analogue CO₂ sensing circuitry, and communicates with the BMP180 over I²C.

The measurements are combined into a timestamped data stream and transmitted wirelessly to an external computer. This separates the sensing hardware from the visualisation system, allowing the musician to carry the relatively small sensing unit while viewing the measurements on a separate device.

Wireless transmission also allows measurements to be logged for later analysis rather than limiting the system to live monitoring.

## 5. Real-Time Visualisation

A Python and PyQt desktop application receives the wireless sensor data and displays the CO₂, pressure and temperature channels in real time.

The application performs additional signal processing and filtering before displaying the measurements, allowing the user to see changes in breath activity as they occur.

Session data can also be buffered for later review, making it possible to compare different playing sessions and examine breath behaviour beyond the immediate live display.

## 6. Potential Musical Application

By capturing quantifiable breath data during real playing, this system opens up several practical applications:

- **Practice feedback** musicians can see how their breath support and timing align with phrasing, dynamics, and endurance across a piece, rather than relying on subjective feel alone.
- **Teaching aid** instructors can use recorded breath data to give concrete, visual feedback on breath control technique, rather than description alone.
- **Endurance and consistency tracking** repeated sessions can be compared over time to track improvements in breath efficiency or stamina for demanding passages.
- **Research** objective breath-activity data across players/instruments could support studies into breath technique and fatigue in wind and brass performance.


## Usage Instructions

The final version of the device is envisioned to work like so:

- Attach mouthpiece to wind instrument and place MYOSA in pocket/hook to belt
- Run accompanying app on phone/computing device
- Allow hardware a few seconds unplayed to calibrate CO2 background level
- Practise instrument as normal, playing along to sheet music on app
- Compare breath activity in app in realtime and/or observe collected data at end of piece

## Tech Stack

### Hardware

| Component | Part | Role |
|---|---|---|
| MCU / wireless | MYOSA development board | Master controller — drives sensing, fuses data, streams wirelessly to host computer/Cloud |
| CO₂ emitter | L15895-0430MA LED | Pulsed mid-wavelength infrared (MWIR) source, tuned to a CO₂ absorption band |
| CO₂ detector | P16112-011MA photodiode | Measures transmitted IR intensity after passing through the breath sample |
| Pressure / temperature | BMP180 (Minkit breakout) | I²C sensor providing an independent breath-pressure signal and temperature compensation |
| Onboard display | 0.96" OLED Display (Minkit breakout) | I²C enabled, providing on device quick insights for potential offline use|
| Mechanical | 3D-printed trombone mouthpiece | Directs exhaled breath through the sensing chamber; mounts sensor package in the airflow path (Based on Popular Bach 12C Design) |
| PCB | Custom board (see `KiCAD/`) | Houses the analog front end and sensor interconnects |

#### Analog Front End

The CO₂ channel is the most sensitive part of the hardware, since NDIR absorption signals are small relative to the raw photodiode output. The front end is built around a **pulsed detection scheme**:

1. **LED pulsing** — the MWIR LED is driven in short pulses (rather than continuously) by the MYOSA board. Pulsing reduces average power draw and, more importantly, lets the readout distinguish the LED's signal from ambient IR and photodiode drift.
2. **Photodiode readout** — the photodiode's current output is converted to a voltage via a transimpedance amplifier (TIA), since photodiode signals are typically too small and too high-impedance to read directly with an ADC.
3. **Filtering** — the amplified signal is filtered to reject noise outside the LED's pulse frequency and suppress ambient light interference.
4. **Synchronous sampling** — the MYOSA board samples the photodiode signal in sync with the LED pulse (on-pulse vs off-pulse), allowing the background/ambient signal to be subtracted from the CO₂-modulated signal. This differential approach is what makes the CO₂ absorption measurable despite a noisy optical environment.
5. **Digitisation** — the conditioned analog signal is read via the MYOSA board's ADC and combined in firmware with the BMP180's I²C readings.

### Software

**Firmware — Arduino C++ (runs on MYOSA)**
- Drives LED pulse timing and synchronises photodiode ADC sampling with each pulse
- Polls the BMP180 over I²C for pressure and temperature
- Fuses CO₂, pressure, and temperature readings into a single timestamped data frame
- Streams the fused data frame wirelessly to the host computer

**Desktop application — Python + PyQt**
- Listens for the incoming data stream from the MYOSA board and parses each frame into CO₂, pressure, and temperature values
- Does enhanced signal processing and filtering
- Plots all three channels live using PyQt's graphing widgets, updating in real time as the musician plays
- Buffers session data for post-session review, and (per the Usage Instructions) supports comparing breath activity against sheet music playback


## Requirements / Installation

**Dependencies** are listed in `requirements.txt` — install with:
```bash
pip install -r requirements.txt
```
**User Interface** run the current user interface using:
```bash
python3 .\Code\app_code\Breathsense_GUI.py
```

## Repository Structure

| Folder | Contents |
|---|---|
| [`Code/`](Code) | Arduino C++ firmware for the MYOSA board (sensor reading, LED pulsing, I²C fusion, wireless streaming) and the Python/PyQt desktop visualisation app |
| [`KiCAD/`](KiCAD) | PCB schematics and layout for the custom analog front end (NDIR CO₂ sensing circuitry) |
| [`Mechanical/`](Mechanical) | 3D-printable mouthpiece adapter design (based on a Bach 12C mouthpiece) that houses the sensor package in the airflow path |

## License MIT License

Copyright (c) 2026 aidanmacro

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.