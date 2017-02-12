#define BOOST_TEST_MODULE DatabaseTests
#include <boost/test/unit_test.hpp>
#include <boost/thread.hpp>

#include "database_comms.h"
#include "spectrum.h"


BOOST_AUTO_TEST_SUITE(Database)

char errorBuf[32];
uint8_t packetBuffer[1500];

void
get_pkt(spectrum *spec, int radio_id)
{
	spectrum_eror_t retVal;
	retVal = spectrum_getPacket(spec, packetBuffer, sizeof(packetBuffer), -1);
}

void
put_pkt(spectrum *spec, int radio_id)
{
	spectrum_eror_t retVal;
   retVal = spectrum_putPacket(spec, packetBuffer, retVal);
}

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
	std::cout << "Stage 3 has started." << std::endl;;

	boost::thread th(boost::bind(launch_database_thread, specTx, radio_number, 100));

	// Everything created. Now simulate TX and RX in the database
	int c = 0;
	while (c++ < 10)
	{
		get_pkt(specTx, radio_number);
		put_pkt(specRx, radio_number);

		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		std::cout << "score: " << DatabaseApi::getInstance().current_score() << std::endl;
	}
}

BOOST_AUTO_TEST_SUITE_END()
