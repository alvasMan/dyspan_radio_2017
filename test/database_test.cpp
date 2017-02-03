#define BOOST_TEST_MODULE DatabaseTests
#include <boost/test/unit_test.hpp>

#include "database_comms.h"


BOOST_AUTO_TEST_SUITE(Database)

BOOST_AUTO_TEST_CASE(test1)
{
    BOOST_CHECK_EQUAL(4, 3);
}

BOOST_AUTO_TEST_SUITE_END()
