########################################################################
# The dyspanradio binary
########################################################################

file(GLOB sources
    main.cpp
    ofdmtransceiver.cpp)

file(GLOB headers
    ofdmtransceiver.h
    channels.h
    Buffer.h)

ADD_EXECUTABLE(dyspanradio ${sources} ${headers})

TARGET_LINK_LIBRARIES(dyspanradio ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
