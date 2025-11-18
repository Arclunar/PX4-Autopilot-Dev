#pragma once
#include <mathlib/mathlib.h>

class StepDistGenerator
{
private:
    float _mag{0.f};
    float _duration{0.f};
    float _freq{0.f};
    float _counter{0.f};

public:
    StepDistGenerator(/* args */) {}
    ~StepDistGenerator() {}

    void setDuration(float duration) {_duration = duration;}
    void setMagnitude(float mag) {_mag = mag;}
    void setFrequency(float freq) {_freq = freq;}

    void reset() {_counter = 0.f;}

    void update(const float &dt, float &dist)
    {
	    _counter += dt;
	//     if (_counter < 0.5f) {
	// 	    dist = 0.f;
	//     } else if (_counter < 0.5f + _duration){
	// 	    dist = - _mag;
	//     } else if (_counter < 0.5f + 2.0f * _duration){
	// 	    dist = _mag;
	//     }
	//     else {
	// 	    dist = 0.f;
	//     }
	if (_counter < _duration) {
		dist = _mag * sinf(2.0f * M_PI_F * _freq * _counter);
	}
	else {
		dist = 0.f;
	}
}
};


