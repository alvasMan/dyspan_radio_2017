#define BOOST_TEST_MODULE DatabaseTests
#include <boost/test/unit_test.hpp>
#include <boost/thread.hpp>

#include "database_comms.h"
#include "spectrum.h"


BOOST_AUTO_TEST_SUITE(Database)

BOOST_AUTO_TEST_CASE(test1)
{
	char errorBuf[32];
	int radio_number;

	/*
	 * Create and connect the transmitter.
	 * Normally this would be on two different machines.
	 */
	spectrum* specTx = spectrum_init(0);
	spectrum_eror_t retVal = spectrum_connect(specTx, "127.0.0.1", 5002, 1500, 1);
	// Error check
	spectrum_errorToText(specTx, retVal, errorBuf, sizeof(errorBuf));
	BOOST_REQUIRE( retVal >= 0);

	/*
	 * Create the receiver
	 */
	spectrum* specRx = spectrum_init(0);
	retVal = spectrum_connect(specRx, "127.0.0.1", 5002, 0, 0);
	// Error check
	spectrum_errorToText(specRx, retVal, errorBuf, sizeof(errorBuf));
	BOOST_REQUIRE( retVal >= 0);

	/*
	 * Get our radio number. Using  RX context gives same result. (It should be 1)
	 */
	radio_number = spectrum_getRadioNumber(specTx);

	/*
	 * Wait for the start of stage 3 (here you get penalized for interference).
	 * The testing database starts in this state so this will instantly return.
	 */
	spectrum_waitForState(specTx, 3, -1);
	spectrum_waitForState(specRx, 3, -1);
	printf("Stage 3 has started.\n");

   DatabaseApi db(10);

   boost::thread th(boost::bind(launch_database_thread, &db, specTx, radio_number));
}

BOOST_AUTO_TEST_SUITE_END()
