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
        self.swtime = (10000,)

    def parse_arguments(self):
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
            else:
                print "ERROR: I don't know this option"

        if len(self.scenario)==1:
            self.swtime = 1000000*1000

        self.print_params()
        self.evaluate_arguments()

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
