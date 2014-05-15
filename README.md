#Pressure Analyzer#

##Authorship##
Created by *Julien Biral*, 2014
Master's Thesis at *CNMAT, UC Berkeley*

##Description##

TO-DO

##TO-DO List##
This project is under construction. The lastest working release is proposed on the Master branch. Other branches are not guaranteed to work and are possibly under construction.

Here's the list of what will be done soon:
- [ ] MATLAB
	- [ ] Maximizing the dynamic of the circuit
	- [ ] Reducing the output impedance at its max
	- [ ] Find the best RC filter
- [ ] Teensy 3.1
	- [ ] Sampling at 100 kHz (Interval Timer)
	- [ ] RF frequencies > MHz --> filter
	- [ ] Circuit is not LTI --> need to take a capacitor high enough to avoid filtering useful frequencies
	- [ ] Filter frequencies above 500 Hz (FIR or IIR?)
	- [ ] Median filter?
	- [ ] Downsample at 500 Hz before sending through serial (via OSC)
- [ ] Sensor
	- [ ] Study of the sensibility