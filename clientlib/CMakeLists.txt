########################################################################
# The clientlib library
########################################################################

file(GLOB clientlibsources
    packetCounters.c
    socket.c
    SpectrumClient.c)

file(GLOB clientlibheaders
    socket.h
    spectrum.h)

#TARGET_LINK_LIBRARIES(dyspanradio packet ${FFTW3F_LIBRARIES} ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${LIQUIDDSP_LIBRARIES})
add_library(clientlib STATIC ${clientlibsources} ${clientlibheaders})
