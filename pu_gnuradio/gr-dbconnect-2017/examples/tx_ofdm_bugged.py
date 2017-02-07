#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: OFDM PU Phy
# Generated: Tue Feb  7 18:37:14 2017
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio import qtgui
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import dbconnect
import pmt
import sip
import sys
import time
from gnuradio import qtgui


class tx_ofdm_bugged(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "OFDM PU Phy")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("OFDM PU Phy")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "tx_ofdm_bugged")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 10e6
        self.taps = taps = filter.firdes.low_pass(1,samp_rate, 0.95e6,0.05e6)
        self.sync_word2 = sync_word2 = [0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 0, 1, -1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1, -1, -1, 0, 0, 0, 0, 0]
        self.sync_word1 = sync_word1 = [0., 0., 0., 0., 0., 0., 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., -1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., -1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 1.41421356, 0., 0., 0., 0., 0., 0.]
        self.pilot_symbols = pilot_symbols = ((1, 1, 1, -1,),)
        self.pilot_carriers = pilot_carriers = ((-21, -7, 7, 21,),)
        self.packet_len = packet_len = 64
        self.occupied_carriers = occupied_carriers = (range(-26, -21) + range(-20, -7) + range(-6, 0) + range(1, 7) + range(8, 21) + range(22, 27),)
        self.len_tag_key = len_tag_key = "packet_len"
        self.interp_factor = interp_factor = 5
        self.fft_len = fft_len = 64
        self.decim_factor = decim_factor = 1

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(('addr=192.168.10.2', "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(1255e6, 0)
        self.uhd_usrp_sink_0.set_gain(15, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.rational_resampler_xxx_0_0_1_1 = filter.rational_resampler_ccc(
                interpolation=interp_factor,
                decimation=decim_factor,
                taps=(taps),
                fractional_bw=None,
        )
        self.rational_resampler_xxx_0_0_1_0 = filter.rational_resampler_ccc(
                interpolation=interp_factor,
                decimation=decim_factor,
                taps=(taps),
                fractional_bw=None,
        )
        self.rational_resampler_xxx_0_0_1 = filter.rational_resampler_ccc(
                interpolation=interp_factor,
                decimation=decim_factor,
                taps=(taps),
                fractional_bw=None,
        )
        self.rational_resampler_xxx_0_0 = filter.rational_resampler_ccc(
                interpolation=interp_factor,
                decimation=decim_factor,
                taps=(taps),
                fractional_bw=None,
        )
        self.qtgui_time_sink_x_0 = Template error: #set $win = 'self._%s_win'%$id
        qtgui.$(type.fcn)(
        	$size, \#size
        	$srate, \#samp_rate
        	$name, \#name
        	#if $type.t == 'message' then 0 else $nconnections# \#number of inputs
        )
        self.$(id).set_update_time($update_time)
        self.$(id).set_y_axis($ymin, $ymax)

        self.$(id).set_y_label($ylabel, $yunit)

        self.$(id).enable_tags(-1, $entags)
        self.$(id).set_trigger_mode($tr_mode, $tr_slope, $tr_level, $tr_delay, $tr_chan, $tr_tag)
        self.$(id).enable_autoscale($autoscale)
        self.$(id).enable_grid($grid)
        self.$(id).enable_axis_labels($axislabels)
        self.$(id).enable_control_panel($ctrlpanel)

        if not $legend:
          self.$(id).disable_legend()

        labels = [$label1, $label2, $label3, $label4, $label5,
                  $label6, $label7, $label8, $label9, $label10]
        widths = [$width1, $width2, $width3, $width4, $width5,
                  $width6, $width7, $width8, $width9, $width10]
        colors = [$color1, $color2, $color3, $color4, $color5,
                  $color6, $color7, $color8, $color9, $color10]
        styles = [$style1, $style2, $style3, $style4, $style5,
                  $style6, $style7, $style8, $style9, $style10]
        markers = [$marker1, $marker2, $marker3, $marker4, $marker5,
                   $marker6, $marker7, $marker8, $marker9, $marker10]
        alphas = [$alpha1, $alpha2, $alpha3, $alpha4, $alpha5,
                  $alpha6, $alpha7, $alpha8, $alpha9, $alpha10]

        #if($type() == "complex" or $type() == "msg_complex")
        for i in xrange(#if $type.t == 'message' then 2 else 2*$nconnections#):
            if len(labels[i]) == 0:
                if(i % 2 == 0):
                    self.$(id).set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.$(id).set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.$(id).set_line_label(i, labels[i])
            self.$(id).set_line_width(i, widths[i])
            self.$(id).set_line_color(i, colors[i])
            self.$(id).set_line_style(i, styles[i])
            self.$(id).set_line_marker(i, markers[i])
            self.$(id).set_line_alpha(i, alphas[i])
        #else
        for i in xrange(#if $type.t == 'message' then 1 else $nconnections#):
            if len(labels[i]) == 0:
                self.$(id).set_line_label(i, "Data {0}".format(i))
            else:
                self.$(id).set_line_label(i, labels[i])
            self.$(id).set_line_width(i, widths[i])
            self.$(id).set_line_color(i, colors[i])
            self.$(id).set_line_style(i, styles[i])
            self.$(id).set_line_marker(i, markers[i])
            self.$(id).set_line_alpha(i, alphas[i])
        #end if

        self._$(id)_win = sip.wrapinstance(self.$(id).pyqwidget(), Qt.QWidget)
        $(gui_hint()($win))
            unsupported operand type(s) for *: 'int' and 'TemplateArg'
        self.digital_ofdm_tx_0_2 = digital.ofdm_tx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  packet_length_tag_key=len_tag_key,
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  rolloff=0,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.digital_ofdm_tx_0_1 = digital.ofdm_tx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  packet_length_tag_key=len_tag_key,
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  rolloff=0,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.digital_ofdm_tx_0_0 = digital.ofdm_tx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  packet_length_tag_key=len_tag_key,
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  rolloff=0,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.digital_ofdm_tx_0 = digital.ofdm_tx(
        	  fft_len=fft_len, cp_len=fft_len/4,
        	  packet_length_tag_key=len_tag_key,
        	  occupied_carriers=occupied_carriers,
        	  pilot_carriers=pilot_carriers,
        	  pilot_symbols=pilot_symbols,
        	  sync_word1=sync_word1,
        	  sync_word2=sync_word2,
        	  bps_header=1,
        	  bps_payload=2,
        	  rolloff=0,
        	  debug_log=False,
        	  scramble_bits=False
        	 )
        self.dbconnect_pktgen_0 = dbconnect.pktgen(1, packet_len, False, False, False, '127.0.0.1', 5002, 5, 10, 2, 20, 50, 100, 30000, 0.05, 6643, 5, 30, 2000, 2)
        self.dbconnect_pdu_fillpath_cpp_0_0_1 = dbconnect.pdu_fillpath_cpp()
        self.dbconnect_pdu_fillpath_cpp_0_0_0 = dbconnect.pdu_fillpath_cpp()
        self.dbconnect_pdu_fillpath_cpp_0_0 = dbconnect.pdu_fillpath_cpp()
        self.dbconnect_pdu_fillpath_cpp_0 = dbconnect.pdu_fillpath_cpp()
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_tagged_stream_to_pdu_0_3_1 = blocks.tagged_stream_to_pdu(blocks.complex_t, 'packet_len')
        self.blocks_tagged_stream_to_pdu_0_3_0 = blocks.tagged_stream_to_pdu(blocks.complex_t, 'packet_len')
        self.blocks_tagged_stream_to_pdu_0_3 = blocks.tagged_stream_to_pdu(blocks.complex_t, 'packet_len')
        self.blocks_tagged_stream_to_pdu_0 = blocks.tagged_stream_to_pdu(blocks.complex_t, 'packet_len')
        self.blocks_pdu_to_tagged_stream_0_0_1 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_pdu_to_tagged_stream_0_0_0 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_pdu_to_tagged_stream_0_0 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_pdu_to_tagged_stream_0 = blocks.pdu_to_tagged_stream(blocks.byte_t, 'packet_len')
        self.blocks_multiply_xx_0_3 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0_2 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0_1 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((0.18, ))
        self.blocks_message_debug_0 = blocks.message_debug()
        self.blocks_add_xx_2 = blocks.add_vcc(1)
        self.analog_sig_source_x_0_3 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, -3.75e6, 1, 0)
        self.analog_sig_source_x_0_2 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, 3.75e6, 1, 0)
        self.analog_sig_source_x_0_1 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, -1.25e6, 1, 0)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, 1.25e6, 1, 0)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0, 'pdus'), (self.dbconnect_pdu_fillpath_cpp_0, 'in0'))
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_3, 'pdus'), (self.dbconnect_pdu_fillpath_cpp_0_0, 'in0'))
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_3_0, 'pdus'), (self.dbconnect_pdu_fillpath_cpp_0_0_0, 'in0'))
        self.msg_connect((self.blocks_tagged_stream_to_pdu_0_3_1, 'pdus'), (self.dbconnect_pdu_fillpath_cpp_0_0_1, 'in0'))
        self.msg_connect((self.dbconnect_pktgen_0, 'cmd'), (self.blocks_message_debug_0, 'print'))
        self.msg_connect((self.dbconnect_pktgen_0, 'out0'), (self.blocks_pdu_to_tagged_stream_0, 'pdus'))
        self.msg_connect((self.dbconnect_pktgen_0, 'out1'), (self.blocks_pdu_to_tagged_stream_0_0, 'pdus'))
        self.msg_connect((self.dbconnect_pktgen_0, 'out2'), (self.blocks_pdu_to_tagged_stream_0_0_0, 'pdus'))
        self.msg_connect((self.dbconnect_pktgen_0, 'out3'), (self.blocks_pdu_to_tagged_stream_0_0_1, 'pdus'))
        self.msg_connect((self.dbconnect_pktgen_0, 'cmd'), (self.uhd_usrp_sink_0, 'command'))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.analog_sig_source_x_0_1, 0), (self.blocks_multiply_xx_0_1, 1))
        self.connect((self.analog_sig_source_x_0_2, 0), (self.blocks_multiply_xx_0_2, 1))
        self.connect((self.analog_sig_source_x_0_3, 0), (self.blocks_multiply_xx_0_3, 1))
        self.connect((self.blocks_add_xx_2, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_add_xx_2, 0))
        self.connect((self.blocks_multiply_xx_0_1, 0), (self.blocks_add_xx_2, 1))
        self.connect((self.blocks_multiply_xx_0_2, 0), (self.blocks_add_xx_2, 2))
        self.connect((self.blocks_multiply_xx_0_3, 0), (self.blocks_add_xx_2, 3))
        self.connect((self.blocks_pdu_to_tagged_stream_0, 0), (self.digital_ofdm_tx_0, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_0_0, 0), (self.digital_ofdm_tx_0_0, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_0_0_0, 0), (self.digital_ofdm_tx_0_1, 0))
        self.connect((self.blocks_pdu_to_tagged_stream_0_0_1, 0), (self.digital_ofdm_tx_0_2, 0))
        self.connect((self.blocks_throttle_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.dbconnect_pdu_fillpath_cpp_0, 0), (self.rational_resampler_xxx_0_0, 0))
        self.connect((self.dbconnect_pdu_fillpath_cpp_0_0, 0), (self.rational_resampler_xxx_0_0_1, 0))
        self.connect((self.dbconnect_pdu_fillpath_cpp_0_0_0, 0), (self.rational_resampler_xxx_0_0_1_0, 0))
        self.connect((self.dbconnect_pdu_fillpath_cpp_0_0_1, 0), (self.rational_resampler_xxx_0_0_1_1, 0))
        self.connect((self.digital_ofdm_tx_0, 0), (self.blocks_tagged_stream_to_pdu_0, 0))
        self.connect((self.digital_ofdm_tx_0_0, 0), (self.blocks_tagged_stream_to_pdu_0_3, 0))
        self.connect((self.digital_ofdm_tx_0_1, 0), (self.blocks_tagged_stream_to_pdu_0_3_0, 0))
        self.connect((self.digital_ofdm_tx_0_2, 0), (self.blocks_tagged_stream_to_pdu_0_3_1, 0))
        self.connect((self.rational_resampler_xxx_0_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.rational_resampler_xxx_0_0_1, 0), (self.blocks_multiply_xx_0_1, 0))
        self.connect((self.rational_resampler_xxx_0_0_1_0, 0), (self.blocks_multiply_xx_0_2, 0))
        self.connect((self.rational_resampler_xxx_0_0_1_1, 0), (self.blocks_multiply_xx_0_3, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "tx_ofdm_bugged")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_taps(filter.firdes.low_pass(1,self.samp_rate, 0.95e6,0.05e6))
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)
        self.analog_sig_source_x_0_3.set_sampling_freq(self.samp_rate)
        self.analog_sig_source_x_0_2.set_sampling_freq(self.samp_rate)
        self.analog_sig_source_x_0_1.set_sampling_freq(self.samp_rate)
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)

    def get_taps(self):
        return self.taps

    def set_taps(self, taps):
        self.taps = taps
        self.rational_resampler_xxx_0_0_1_1.set_taps((self.taps))
        self.rational_resampler_xxx_0_0_1_0.set_taps((self.taps))
        self.rational_resampler_xxx_0_0_1.set_taps((self.taps))
        self.rational_resampler_xxx_0_0.set_taps((self.taps))

    def get_sync_word2(self):
        return self.sync_word2

    def set_sync_word2(self, sync_word2):
        self.sync_word2 = sync_word2

    def get_sync_word1(self):
        return self.sync_word1

    def set_sync_word1(self, sync_word1):
        self.sync_word1 = sync_word1

    def get_pilot_symbols(self):
        return self.pilot_symbols

    def set_pilot_symbols(self, pilot_symbols):
        self.pilot_symbols = pilot_symbols

    def get_pilot_carriers(self):
        return self.pilot_carriers

    def set_pilot_carriers(self, pilot_carriers):
        self.pilot_carriers = pilot_carriers

    def get_packet_len(self):
        return self.packet_len

    def set_packet_len(self, packet_len):
        self.packet_len = packet_len

    def get_occupied_carriers(self):
        return self.occupied_carriers

    def set_occupied_carriers(self, occupied_carriers):
        self.occupied_carriers = occupied_carriers

    def get_len_tag_key(self):
        return self.len_tag_key

    def set_len_tag_key(self, len_tag_key):
        self.len_tag_key = len_tag_key

    def get_interp_factor(self):
        return self.interp_factor

    def set_interp_factor(self, interp_factor):
        self.interp_factor = interp_factor

    def get_fft_len(self):
        return self.fft_len

    def set_fft_len(self, fft_len):
        self.fft_len = fft_len

    def get_decim_factor(self):
        return self.decim_factor

    def set_decim_factor(self, decim_factor):
        self.decim_factor = decim_factor


def main(top_block_cls=tx_ofdm_bugged, options=None):
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
