find_library(PYTHON_LIBRARY 
            NAMES python2.7
            HINTS $ENV{PYTHON_DIR}/lib
            PATHS /usr/local/lib
                  /usr/lib)

set(PYTHON_LIBRARIES ${PYTHON_LIBRARY})

