########################################################################
# The dyspanradio binary
########################################################################

file(GLOB sources
    main.cpp
    ofdmtransceiver.cpp
    multichannelrx.cpp
    multichannelrx_pfb.cpp
    EnergyDetector.cpp)

file(GLOB headers
    dyspanradio.h
    ofdmtransceiver.h
    multichannelrx.h
    multichannelrx_pfb.h
    EnergyDetector.h
    channels.h
    Buffer.h
    buffer_factory.h)

ADD_EXECUTABLE(dyspanradio ${sources} ${headers})

TARGET_LINK_LIBRARIES(dyspanradio ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
