########################################################################
# The dyspanradio binary
########################################################################

file(GLOB sources
    main.cpp
    ofdmtransceiver.cpp
    EnergyDetector2.cpp
    EnergyDetector.cpp)

file(GLOB headers
    ofdmtransceiver.h
    EnergyDetector2.h
    EnergyDetector.h
    channels.h
    Buffer.h)

ADD_EXECUTABLE(dyspanradio ${sources} ${headers})

TARGET_LINK_LIBRARIES(dyspanradio ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
