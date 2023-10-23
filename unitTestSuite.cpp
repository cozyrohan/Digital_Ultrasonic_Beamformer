/*
 * unitTestSuite.cpp
 *
 *  Created on: Oct 22, 2023
 *      Author: cozyrohan
 */


#include "unitTestSuite.hpp"
#include "testClass.hpp"
#include <cassert>





TestClass tc{343, 39000, 7, 0, 0};

bool UnitTestSuite::test_0_test_deg_rad_conversion()
{
	assert((1==1));


	return true;
}

bool UnitTestSuite::test_1_test_rad_deg_conversion()
{
	assert((1==1));


	return true;
}

bool UnitTestSuite::test_2_test_wavelength_calc()
{
	assert((1==1));


	return true;
}

bool UnitTestSuite::test_3_test_distance_calc()
{
	assert((1==1));


	return true;
}

bool UnitTestSuite::test_4_test_time_delay_calc()
{
	assert((1==1));


	return true;
}




bool UnitTestSuite::test_all()
{
	for(int i = 0; i < num_tests; i++)
	{
		//must use this because member functions NEED instances to be called on (not static)
		(this->*UnitTestSuite::tests[i])();
	}


	return true;
}





