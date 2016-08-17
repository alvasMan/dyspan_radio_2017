dyspanradio
===========

dyspanradio is the implementation of a frequency-agile SDR used at
the IEEE DySpan 2015 conference during the Spectrum Sharing Challenge.

It is written in C++ and builds upon the liquid-dsp library (http://liquidsdr.org/)
for its PHY layer.


Disclaimer
----------

dyspanradio is provided with NO WARRANTY OF ANY KIND. Users of this software are expected to comply with all applicable local, national and international telecom and radio spectrum regulations.


License
-------

dyspanradio is released under the AGPLv3 license.

Copyright
---------

dyspanradio contains code snippets from the examples shipped with liquid-dsp (X11/MIT)
as well code from the organizers (clientlib) to connect to the provided facilities (no license provided).
It also uses a readerwriterqueue implementation of Cameron Desrochers (BSD) which
uses Jeff Preshing's semaphore implementation (zlib license).
