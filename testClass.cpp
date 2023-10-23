/*
 * testClass.cpp
 *
 *  Created on: Oct 21, 2023
 *      Author: cozyrohan
 */

#include "testClass.hpp"
#include <math.h>
//  TestClass tc{343, 39000, 7, 0, 0};
//  double td = tc.calc_time_delay_amount(30);

TestClass::TestClass(double sos, int freq, int num_t, double wl, double dist_t )
{

 /*
    double speed_of_sound = 343; 						// 	shpeed of sound								(m/s)

	double frequency = 39000;						// frequency of the wave						(Hz)

	int number_transducers = 7; 						// the size of our transmit array 				(count)

	double wavelength = 0;		// **set this again by calling set_wavelength    (m)

	double d = 0;  							// distance bw centers of adjacent transducers 	(m)
  */

	speed_of_sound = sos;
	frequency = freq;
	number_transducers = num_t;
	wavelength = wl;
	d = dist_t;
}

void TestClass::set_wavelength()
{
	wavelength = speed_of_sound/frequency;
}

void TestClass::set_transducer_distance()
{
	d = wavelength/2;
}

double TestClass::deg_to_rad(double deg)
{
	return deg * M_PI /180;
}

double TestClass::rad_to_deg(double rad)
{
	return rad * 180 / M_PI;
}

double TestClass::calc_time_delay_amount(double theta)
{
	// return the calculated amount of delay for the transducer t[1]

	// @param 	theta : in rad (or deg?)
	// @return	delay : in microsec


	// phase_shift = 360deg * d * sin(theta) / wavelength

	theta = deg_to_rad(theta);

	double phase_shift_rad = 2.0 * M_PI * sin(theta) / wavelength;

	int phase_shift_deg = rad_to_deg(phase_shift_rad);

	phase_shift_deg = phase_shift_deg % (360); // leverage periodicity

	double time_del = phase_shift_deg / (360.0 * frequency);

	return time_del * 1000000;
}


