# A7 - Frequency Measurement

## Reference Materials
- STM32L4xxxx Reference Manual (RM) – ADC, General Purpose Timers, Comparator
- STM32L4x6xx Datasheet (DS) – Pinouts
- NUCLEO-L4x6RG Users Manual (UM)

## Signal Measurement
Measuring input signals with an MCU typically involves more than just monitoring constant values or DC voltages. When analyzing different signals, characteristics beyond just the voltage value at a singular point are important. One of the most common aspects of a signal to characterize is its frequency. This is because properly sampling a signal is easier if the frequency is first known. If the signal has a frequency of 1kHz, sampling at a rate of 1 ms will be inadequate because only 1 data point per period is insufficient for any type of signal analysis. Sampling at 10 us will provide 100 points per period. If the signal was 10 Hz, sampling at this same rate of 10 us will require 10000 samples to capture the full period. Trying to analyze that many points would be computationally expensive, especially when similar precision results could be achieved with orders of magnitude fewer data points. The limited memory on a microcontroller will also limit the number of samples that can be saved.  
This assignment will help practice and prepare for the next project by developing a strategy to measure the frequency of an input signal. Determining the frequency requires collecting multiple data points at consistent time intervals for analysis. To achieve quality measurements, calibrated use of timers in coordination with the ADC is needed.

## Instructions
- Expand on the previous ADC assignment to sample an analog input signal and determine its frequency. 
- Develop your own software solution or use a comparator (internal or external) with the timer input capture to measure the period of an input wave. Using an FFT will not be acceptable for this assignment.
- Display the result in Hz on a terminal via UART. 
  - The value on the terminal should be updated at least every 2 seconds
- For this assignment, the input signals will be limited to the following
    - Input signals will have a sinusoidal waveform
    - Input signal frequency will vary from 1 Hz to 1 kHz
    - The maximum voltage of any input signal will be 3 V
    - The minimum voltage of any input signal will be 0 V
    - The minimum peak-to-peak voltage of any input signal will be 0.5 V
    - The maximum DC offset value of any input signal will be 2.75 V

## Hints
- Speed up the MCU, running at a minimum of 24 MHz. 
- Run the ADC clock at the fastest clock rate to make the conversion process as quick as possible. Adjust the desired sample and hold time of the ADC accordingly.
- Use a timer to set the start conversion bit at regular intervals for uniform sampling.
- Building some circuits in hardware can be helpful and reduce the complexity of the software.
- An analog circuit that could transform a periodic wave into a square wave of the same frequency can be used by a timer in capture mode to accurately measure the period.
- The internal comparator peripheral in the STM32L4 can also be used with a simpler analog circuit, the DAC, or internal reference generator to achieve a similar result. 
- Adding hardware is not inherently easier or harder, it is just a different approach with different trade-offs. Adding hardware can reduce the complexity of the software that needs to be debugged and verified. However, the added hardware will require debugging and verification which a purely software approach would not.

## Demonstration
- Demonstrate by showing the minimum and maximum frequencies that can be measured (1 Hz to 1 kHz for full credit) with each input signal specified below
    - 3 Vpp sine wave with 1.5 V DC offset
    - 0.5 Vpp sine wave with 0.5 V DC offset
    - 1 Vpp sine wave with 2.5 V DC offset

## Deliverables
- Demonstrate your working program to the instructor or lab assistant
- Create a single pdf document (not a full lab report) containing the following:
- Schematic of your working device.
- Properly formatted C source code
