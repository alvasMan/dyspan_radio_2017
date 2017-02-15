########################################################################
# The dyspanradio binary
########################################################################

file(GLOB sources
    main.cpp
    ofdmtransceiver.cpp
    multichannelrx.cpp
    multichannelrx_pfb.cpp
    ChannelPowerEstimator.cpp
    sensing_components.cpp
    context_awareness.cpp
    database_comms.cpp
    modulation_search_api.cpp
    json_utils.cpp
    markov_chain_components.cpp)

file(GLOB headers
    dyspanradio.h
    ofdmtransceiver.h
    multichannelrx.h
    multichannelrx_pfb.h
    ChannelPowerEstimator.h
    channels.h
    Buffer.h
    buffer_factory.h
    packetLib.h)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/clientlib)
ADD_EXECUTABLE(dyspanradio ${sources} ${headers})
TARGET_LINK_LIBRARIES(dyspanradio clientlib ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES} ${PYTHON_LIBRARIES})
