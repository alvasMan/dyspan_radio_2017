#!/usr/bin/env python2

import sys, getopt

class Params:
    def __init__(self):
        # these are the default values
        self.db_ip="127.0.0.1"
        self.scenario = -1
        self.channel1 = -1
        self.channel2 = -1
        self.gain = -1
        self.gain_period = 1000
        self.frequency = 1.255e9
        self.antenna = "TX/RX"
        self.swtime = (10000,)
        self.func_scripts = {"tx_ofdm_newv":self.process_tx_ofdm_newv,"tx_ofdm":self.process_tx_ofdm,"rx_ofdm":self.process_rx_ofdm}

    def parse_arguments(self, script_str = "tx_ofdm_newv"):
        help_str = '<executable> -d <database ip> -s <scenario number> -c <"channel1,channel2"> -g <gain>'
        try:
            opts, args = getopt.getopt(sys.argv[1:],"hd:s:c:g:s:",["db_ip=","scenario=","channels=","gain=","swelltime="])
        except getopt.GetoptError:
            if len(sys.argv)>1:
                print help_str
                sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                print help_str
                sys.exit()
            elif opt in ("-d", "--db_ip"):
                self.db_ip = arg
            elif opt in ("-s", "--scenario"):
                spl = [x.strip() for x in arg.split(',')]
                self.scenario = [int(x) for x in spl]
            elif opt in ("-c", "--channels"):
                spl = [x.strip() for x in arg.split(',')]
                self.channel1 = int(spl[0])
                if len(spl)>1:
                    self.channel2 = int(spl[1])
            elif opt in ("-g", "--gain"):
                spl = [x.strip() for x in arg.split(',')]
                self.gain = [int(x) for x in spl]
            elif opt in ("-s", "--swelltime"):
                spl = [x.strip() for x in arg.split(',')]
                self.swtime = [x for x in spl]
            elif opt in ("-f", "--frequency"):
                self.frequency = float(arg)
            elif opt in ("-a", "--antenna"):
                self.antenna = arg
            else:
                print "ERROR: I don't know this option"

        if len(self.scenario)==1:
            self.swtime = 1000000*1000

        self.funcs_scripts[tx_ofdm_newv]()

        self.process_params()
        self.print_params()
        self.evaluate_arguments()

    def process_rx_ofdm(self):
        fail_tests = [False]*1
        fail_tests[0] = len(self.gain)>1
        if self.gain<0:
            self.gain = 15
        if any(tests):
            print "ERROR: The provided rx_ofdm parameters are not valid"

    def process_tx_ofdm(self):
        if pu_params.gain < 0:
            pu_params.gain=range(5,30)
        pass

    def process_tx_ofdm_newv(self):
        if pu_params.scenario<0:
            pu_params.scenario = range(0,10)
        if pu_params.gain<0:
            pu_params.gain = range(0,30)
        pass

    def process_param(self):
        if self.scenario == -1 or len(self.scenario)>1:
            self.gain_period = 1000000000 # never change gain if scenario is more than one

    def evaluate_arguments(self):
        scenarios_4_channels = [3,5,6,7,8,9,-1]
        scenarios_2_channels = [2,4]
        scenarios_1_channels = [0,1]
        tests = [True]*3
        tests[0] = self.channel1 < 0 or self.channel2 < 0 or self.channel1 != self.channel2
        #tests[0] = self.channel1 >= 0 and self.channel2 >= 0 and self.scenario in scenarios_2_channels
        #tests[1] = self.channel1 >= 0 and self.channel2 < 0 and self.scenario in scenarios_1_channels
        #tests[2] = self.channel1 < 0 and self.channel2 < 0 and self.scenario in scenarios_4_channels
        if not any(tests):
            print "ERROR: The provided configuration is not valid."
            sys.exit()

    def print_params(self):
        print 'Database IP: ', self.db_ip
        print 'Scenario: ', self.scenario
        print 'Channels: ', [self.channel1, self.channel2]
        print 'Gain: ', self.gain
