#ifndef FILTERFIR_H
#define FILTERFIR_H

#include <cstdint>

template<int filterTaps>
class FIR {

private:
	int16_t values[filterTaps];	
	int16_t coef[filterTaps];
	float gain; // set to 1 and input unity to see what this needs to be	
	int k; // k stores the index of the current array read to create a circular memory through the array

public:
	FIR(){
		k = 0; //initialize so that we start to read at index 0
		for (int i=0; i<filterTaps; i++) {      
			values[i] = 0; // to have a nice start up, fill the array with 0's
		}
		//TODO calculate default gain?
		//TODO calculate default coefs?
	}
	
	FIR(float newGain, float *newCoefs) {
		k = 0; //initialize so that we start to read at index 0
		setGain(newGain);
		for (int i=0; i<filterTaps; i++) {      
			values[i] = 0; // to have a nice start up, fill the array with 0's
		}
		setCoefficients(newCoefs);
	}

	void setGain(float newGain) {
		gain = newGain;
	}
	
	void setCoefficients(const int32_t *newCoefs) {
		for (int i=0; i<filterTaps; i++) {      
			coef[i] = newCoefs[i];
		}
	}
	
	void setCoefficient(int idx, float newCoef) { 
		coef[idx] = newCoef; 
	}
	
	float process(int16_t in) {
		int32_t out = 0;                        // out is the return variable. It is set to 0 every time we call the filter!

		values[k] = in;                        // store the input of the routine (contents of the 'in' variable) in the array at the current pointer position

		for (int i=0; i<filterTaps; i++) {                              // we step through the array
			out += (int32_t) coef[i] * (int32_t) values[(i + k) % filterTaps];      // ... and add and multiply each value to accumulate the output
																					//  (i + k) % filterTaps creates a cyclic way of getting through the array
		}

		out /= gain;                        // We need to scale the output (unless the coefficients provide unity gain in the passband)

		k = (k+1) % filterTaps;            // k is increased and wraps around the filterTaps, so next time we will overwrite the oldest saved sample in the array

		return out;                              // we send the output value back to whoever called the routine
	}
};
#endif