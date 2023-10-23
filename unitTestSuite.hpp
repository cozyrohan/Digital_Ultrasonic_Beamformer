/*
 * unitTestSuite.hpp
 *
 *  Created on: Oct 22, 2023
 *      Author: cozyrohan
 */

#ifndef SRC_UNITTESTSUITE_HPP_
#define SRC_UNITTESTSUITE_HPP_


class UnitTestSuite
{
public:
	typedef bool (UnitTestSuite::*TestFunc)();

	int num_tests = 1;
	TestFunc tests[5] = {		&UnitTestSuite::test_0_test_deg_rad_conversion,
								&UnitTestSuite::test_1_test_rad_deg_conversion,
								&UnitTestSuite::test_2_test_wavelength_calc,
								&UnitTestSuite::test_3_test_distance_calc,
								&UnitTestSuite::test_4_test_time_delay_calc};

	// ALL TEST RETURN TRUE FOR PASS; FALSE OTHERWISE

	bool test_0_test_deg_rad_conversion();

	bool test_1_test_rad_deg_conversion();

	bool test_2_test_wavelength_calc();

	bool test_3_test_distance_calc();

	bool test_4_test_time_delay_calc();



	bool test_all();


};


#endif /* SRC_UNITTESTSUITE_HPP_ */
