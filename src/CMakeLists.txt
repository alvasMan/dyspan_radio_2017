########################################################################
# The dyspanradio binary
########################################################################

file(GLOB sources
    main.cpp
    ofdmtransceiver.cpp
    ofdmreceiver.cpp
    multichannelrx.cpp
    EnergyDetector.cpp)

file(GLOB headers
    dyspanradio.h
    ofdmtransceiver.h
    ofdmreceiver.h
    multichannelrx.h
    EnergyDetector.h
    channels.h
    Buffer.h
    buffer_factory.h)

ADD_EXECUTABLE(dyspanradio ${sources} ${headers})

TARGET_LINK_LIBRARIES(dyspanradio ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
