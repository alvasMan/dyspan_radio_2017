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
    buffer_factory.h
    packetLib.h)

ADD_EXECUTABLE(dyspanradio ${sources} ${headers})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--unresolved-symbols=ignore-in-object-files")
TARGET_LINK_LIBRARIES(dyspanradio packet ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
