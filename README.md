# OpenROV Underwater Acoustic Location System Project Overview
Our project is the Underwater Acoustic Location System (UALS) for the OpenROV platform. The project consists of 3 surface vessel mounted transponders that send out bursts of acoustic signals at different frequencies. The OpenROV’s onboard receiver will convert these signals using an ADC. After the signals are converted to digital then they are processed to calculate when each signal arrived relative to each other at the OpenROV. With this data as well as other sensors on the OpenROV, an accurate 3D location can be calculated on the computer onboard the surface vessel. This location data will be useful to the operator, and is beneficial to eventually fully-autonomous underwater vehicles that are inexpensive.

# Approach
We will use 3 piezoelectric transducers to convert a sine wave generated on the vessel into 3 varying frequency acoustic signals. The signals will be in the 30-40 kHz range, and will all be generated at the same time. The receiver and analog to digital converter board will feed a  digital representation of the received acoustic signals  (sampled at 220 Ksps) to the Cortex M4 board for signal processing.

The Cortex M4 will differentiate each of the signals by frequency, and determine at which time each signal first reached the OpenROV. The 3 signal’s arrival times will be sent up the tether of the OpenROV to a computer on surface vessel in which the signals departure time - the 3 arrival times can give the overall time required for each signal to reach the OpenROV. With the speed of sound underwater, (which is dependent on both salinity and temperature of water), the distance from each of the 3 transponders to the OpenROV can be calculated. After the distance from each transponder is determined, 3D location can be determined with the help of the depth sensor on board the OpenROV. This location can then be displayed to the user.

Our team members are James Smith, Luis Sanchez and Jason Shen. James and Luis will be working on creating the DSP algorithm to process the incoming signals, and Jason will be working on creating test cases to simulate real-world data.

# Objectives
Working C implementation of the signal processing algorithm that runs on a Cortex M4 (STM32F4463E) Nucleo) microcontroller by the end of the quarter to determine receiving time of 3 acoustic signals

Deliverables
============
Deliverables will include the code that will be able to take in the signals after they are converted into digital. The code will implement filters to identify the correct sources of the beacon signals, and be able to compute relative times that will be then sent to the surface vessel via socket.io interface. Potential long term goals include adding a graphical display for the current location of the OpenROV on the surface vessel computer, after the triangulation is calculated.

1. C code that given a frequency range, filters out all signals at frequencies out of that range. Code should be modular to be able to handle different transmitting frequencies and should use DSP functionality provided by CMSIS.
2. C code that given a sample, can determine when the beginning of a pulse occurred. Code should be modular to handle different signal times (# of pulses, sine vs square wave)
3. Send data to surface computer via socket.io interface
4. Code to be run of surface computer to determine given arrival times of signals, what is location of OpenROV?

# Constraints, Risks and Feasibility
1. Risks
  * One possible risk is that the algorithm we develop works on artificial test data, but when we receive actual data, it works suboptimally. This is why the creation of artificial data must be streamlined, and have many parameters to create more “realistic” data.
  * Another risk is that our algorithm might require more processing power that the Cortex M4 can provide. In this case, we may have to port our algorithm to another board, possibly an FPGA or DSP.
2. The major roles are algorithm development, algorithm testing and communications (to surface vessel). Decisions will be made by consensus, which will be easier because we have 3 members. Also, we can consult Jim for help on technical decisions we need to make down the line. We will communicate via group message, and will have face-to-face meetings twice per week (Mondays and Thursdays). We will also use a git repository for source code management. We will know we are off schedule when we miss a milestone deadline. To deal with schedule slips, we will need to work harder to get back on track or possibly reanalyze our deadlines to determine if they are feasible. James will be doing the weekly status reports.

# Project Development
1. Development roles
  * Developing Algorithm - James, Luis (we need to break this down further)
  * Testing - Jason
2. Hardware
  * We will be starting with STM32F446 Nucleo-64 (Cortex M4), which we currently don’t have available. It’s cost is $10. We will order it from ST and it should take about a week for it to arrive.
3. Software
  * We will be using the C program language and Python for creating test signals to process.
4. Milestones
  * | James | Luis | Jason
    ----|----|----|----
    Week 4 | Understand Filtering Techniques (FIR, fft, bandpass), Setup Github
    Week 5 | Implement Hello World on STM32, debug | same | same
    Week 6/7 | Setup ADC so that we can buffer last 10ms of signal | Optimize filter parameters in Python | Optimize filter parameters in Python
    Week 8 | Get DAC working on separate microcontroller to create test signals | Interface ADC code with algorithm | Create test signals, adding noise similar to what will be encountered underwater
    Week 9/10 | Run tests and Fix Bugs/ Stretch Milestones | same | same


# Weekly Deliverables
###### Week 5:

Understanding Filtering Techniques - Write report on possible filtering techniques Due monday. Defining fft, fir filter, bandpass filtering Week 5

###### Week 6:

Implement Hello World on STM32 - Get toolchain setup for STM32 and get example such as blinking an LED working on board  

###### Week 7:

Script filter parameter optimizations (Luis) - Create script that generates filter parameters and determines error of different points in 3D space to optimize which parameters to use for final product

ADC buffer (James): Working demo or report on how the ADC buffer would work.

Port algorithm to C (Jason): Have working C code that performs the signal processing on the STM32 board.

###### Week 8:

Create test signals (Jason) - generate artificial test data by setting up time difference from 3 transponders, and adding the signals that the OpenROV would receive. Next, add noise by adding random other signals or possibly from underwater recorded audio and determine if our algorithm can accurately determine the time difference of arrival. Due Monday Week 9.??

DAC microcontroller for testing (James): Report or working demo of the integration of the onboard DAC along with our test suite.

Interface ADC code with algorithm (Luis): Working demo of the ADC code and the algorithm working.

###### Week 9:

Convert Filter Code to run on STM32 - not a direct translation, because we want to optimize our code in C using the DSP functions provided by CSMIS. Due Monday Week 9.

# Stretch Milestones

Communication using socket.io - sending TDOA data to surface computer.

Location Triangulation - calculating the 3D location using TDOA data and depth sensor, possibly Kalman Filter.

# Team Members
James Smith, Luis Sanchez, Jason Shen

Based off from work done by Jim Trezzo jim@openrov.com
https://github.com/jtrezzo/acoustic-location-system
