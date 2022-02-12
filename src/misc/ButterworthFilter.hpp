#pragma once

#include <cmath>

// Second order butterworth filter
class ButterworthFilter
{
public:

	ButterworthFilter(float sample_freq, float cutoff_freq)
	{
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}

	// Change filter parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq)
	{
		_cutoff_freq = cutoff_freq;

		// reset delay elements on filter change
		_delay_element_1 = 0.0f;
		_delay_element_2 = 0.0f;

		if (_cutoff_freq <= 0.0f) {
			// no filtering
			_b0 = 1.0f;
			_b1 = 0.0f;
			_b2 = 0.0f;

			_a1 = 0.0f;
			_a2 = 0.0f;

			return;
		}

		const float fr = sample_freq / _cutoff_freq;
		const float ohm = std::tan(M_PI / fr);
		const float c = 1.0f + 2.0f * std::cos(M_PI / 4.0f) * ohm + ohm * ohm;

		_b0 = ohm * ohm / c;
		_b1 = 2.0f * _b0;
		_b2 = _b0;

		_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
		_a2 = (1.0f - 2.0f * std::cos(M_PI / 4.0f) * ohm + ohm * ohm) / c;
	}

	// Add a new raw value to the filter
	float apply(float sample)
	{
		// do the filtering
		float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;

		if (!std::isfinite(delay_element_0)) {
			// don't allow bad values to propagate via the filter
			delay_element_0 = sample;
		}

		const float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		// return the value. Should be no need to check limits
		return output;
	}

	// Reset the filter state to this value
	float reset(float sample)
	{
		const float dval = sample / (_b0 + _b1 + _b2);

		if (std::isfinite(dval)) {
			_delay_element_1 = dval;
			_delay_element_2 = dval;

		} else {
			_delay_element_1 = sample;
			_delay_element_2 = sample;
		}

		return apply(sample);
	}

private:

	float _cutoff_freq{0.0f};

	float _a1{0.0f};
	float _a2{0.0f};

	float _b0{0.0f};
	float _b1{0.0f};
	float _b2{0.0f};

	float _delay_element_1{0.0f};	// buffered sample -1
	float _delay_element_2{0.0f};	// buffered sample -2
};
