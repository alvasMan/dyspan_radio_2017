########################################################################
# The dyspanradio binary
########################################################################

file(GLOB sources
    main.cpp
    ofdmtransceiver.cpp)

ADD_EXECUTABLE(dyspanradio ${sources})

TARGET_LINK_LIBRARIES(dyspanradio ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
