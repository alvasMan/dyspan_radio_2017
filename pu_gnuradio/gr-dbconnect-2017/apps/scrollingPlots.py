import initExample 
import paho.mqtt.client as mqtt

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import averager as avg 

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("spectrum_challenge/#")

pu_bytes=1
su_bytes=1
otpu=1
prev_pu_bytes=1
prev_su_bytes=1
prev_time=0
cur_time=1
start_time=0
score_su=0
only_once = True
stage=0
prev_thpu=0
prev_thsu=0

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global pu_bytes,su_bytes, otpu, cur_time, start_time, only_once, stage 
    data = msg.payload.split(",")
    if data[0] == '5002':
        pu_bytes = int(data[6]) 
        otpu=int(data[12].strip("\r"))
    else:
        su_bytes = int(data[6]) 
    cur_time=int(data[2])+int(data[3])*1e-9
    if int(data[1])== 3 and only_once:
        start_time=int(data[2])+int(data[3])*1e-9
        only_once = False
    stage = data[1]
    #print(msg.topic+" "+str(data))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.5.1", 1883, 60)
#client.loop(timeout=0.1,max_packets=10000)
client.loop_start()

s = 0
m = 0
h = 0
 
class StopWatch(QtGui.QMainWindow):
 
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.initUI()
 
    def initUI(self):
 
        centralwidget = QtGui.QWidget(self)
 
        self.lcd = QtGui.QLCDNumber(self)
        self.lcd.setSegmentStyle(QtGui.QLCDNumber.Flat)
        # get the palette
        palette = self.lcd.palette()

        # foreground color
        palette.setColor(palette.WindowText, QtGui.QColor(85, 85, 255))
        # background color
        palette.setColor(palette.Background, QtGui.QColor(0, 170, 255))
        # "light" border
        palette.setColor(palette.Light, QtGui.QColor(255, 0, 0))
        # "dark" border
        palette.setColor(palette.Dark, QtGui.QColor(0, 255, 0))

        # set the palette
        self.lcd.setPalette(palette)
 
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.Time)
 
        self.start = QtGui.QPushButton("Start",self)
        self.start.clicked.connect(self.Start)
 
        self.stop = QtGui.QPushButton("Stop",self)
        self.stop.clicked.connect(lambda: self.timer.stop())
 
        self.reset = QtGui.QPushButton("Reset",self)
        self.reset.clicked.connect(self.Reset)
 
        grid = QtGui.QGridLayout()
         
        grid.addWidget(self.start,1,0)
        grid.addWidget(self.stop,1,1)
        grid.addWidget(self.reset,1,2)
        grid.addWidget(self.lcd,2,0,1,3)
 
        centralwidget.setLayout(grid)
 
        self.setCentralWidget(centralwidget)
 
#---------Window settings --------------------------------
         
        self.setGeometry(300,300,280,170)
 
    def Reset(self):
        global s,m,h
 
        self.timer.stop()
 
        s = 0
        m = 0
        h = 0
 
        time = "{0}:{1}:{2}".format(h,m,s)
 
        self.lcd.setDigitCount(len(time))
        self.lcd.display(time)
 
    def Start(self):
        global s,m,h
         
        self.timer.start(1000)
     
    def Time(self):
        global s,m,h
 
        if s < 59:
            s += 1
        else:
            if m < 59:
                s = 0
                m += 1
            elif m == 59 and h < 24:
                h += 1
                m = 0
                s = 0
            else:
                self.timer.stop()
 
        time = "{0}:{1}:{2}".format(h,m,s)
 
        self.lcd.setDigitCount(len(time))
        self.lcd.display(time)

pg.mkQApp()
pg.setConfigOption('background','w')
pg.setConfigOption('foreground','k')
tab = pg.QtGui.QTabWidget()
tab.show()
tab.resize(600, 600)
win = pg.GraphicsWindow()
grid = QtGui.QGridLayout()
qwid = QtGui.QWidget()
win.setWindowTitle('Spectrum challenge')
sw= StopWatch()
scroll= tab.addTab(qwid,"Throughput")
qwid.setLayout(grid)
split1 = QtGui.QSplitter()
grid.addWidget(split1)

split1.addWidget(win)
split1.addWidget(sw)
split1.setOrientation(0);


w = pg.TableWidget(editable=True,sortable=True)
bsave = QtGui.QPushButton("Save")
bsave.clicked.connect(w.saveAll)
scwin = pg.GraphicsWindow()
scgrid = QtGui.QGridLayout()
scqwid = QtGui.QWidget()
#w.show()
#w.resize(500,500)
#w.setWindowTitle('pyqtgraph example: TableWidget')
score = tab.addTab(scqwid,"Scorecard")
scqwid.setLayout(scgrid)
split2 = QtGui.QSplitter()
scgrid.addWidget(split2)
split2.addWidget(scwin)
split2.addWidget(w)
split2.addWidget(bsave)
split2.setOrientation(0);


data = np.array([
        (1,'KIT', 0  ),
        (1,'AIT', 0 ),
        (1,'FORTH-ICS',  0),
        (1,'CTVR/CONNECT', 0),
        (1,'FR', 0),
        (2,'KIT', 0  ),
        (2,'AIT', 0 ),
        (2,'FORTH-ICS',  0),
        (2,'CTVR/CONNECT', 0),
        (2,'FR', 0),
], dtype=[('Run', int), ('Team', object), ('Score', float)])

#data=np.loadtxt("scores.txt",dtype=[('Run', int), ('Team', object), ('Score', float)])

w.horizontalHeader().setStyleSheet("QHeaderView { font-size: 22pt; }")
w.setData(data)
w.setEditable(True)

changingLabel = QtGui.QLabel()
changingLabel.setMinimumWidth(200)
font = changingLabel.font()
font.setBold(True)
font.setPointSize(14)
changingLabel.setFont(font)
palette = QtGui.QPalette()
palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.red)
changingLabel.setPalette(palette)

def valueChanging(value):
        changingLabel.setText("Score: %s" % str(value))

win.nextRow()
p4 = win.addPlot()
p4.showGrid(x=True, y=True)
p4.setDownsampling(mode='peak')
p4.setClipToView(True)
curve4 = p4.plot(pen='r', fillLevel=0, brush=(255, 0, 0, 80))
curve4_2 = p4.plot(pen='b',fillLevel=0, brush=(0, 0, 255, 80))
pu = pg.PlotDataItem(pen='r')
su = pg.PlotDataItem(pen='b')
legend = p4.addLegend()
legend.addItem(pu, "Primary User")
legend.addItem(su, "Secondary User")

data3 = np.empty(100)
data3_2 = np.empty(100)
ptr3 = 0

def update2(newdata1, newdata2):
    global data3,data3_2, ptr3
    #data3[ptr3] = data3[ptr3-1]+newdata1
    #data3_2[ptr3] = data3_2[ptr3-1]+newdata2
    data3[ptr3] = newdata1
    data3_2[ptr3] = newdata2
    ptr3 += 1
    if ptr3 >= data3.shape[0]:
        tmp = data3
        data3 = np.empty(data3.shape[0] * 2)
        data3[:tmp.shape[0]] = tmp
        tmp = data3_2
        data3_2 = np.empty(data3_2.shape[0] * 2)
        data3_2[:tmp.shape[0]] = tmp
    #curve3.setData(data3[:ptr3])
    #curve3.setPos(-ptr3, 0)
    curve4.setData(data3[:ptr3])
    curve4_2.setData(data3_2[:ptr3])


# 3) Plot in chunks, adding one new plot curve for every 100 samples
chunkSize = 100
# Remove chunks after we have 10
maxChunks = 10
startTime = pg.ptime.time()
win.nextRow()
p5 = win.addPlot(colspan=2)
p5.showGrid(x=True, y=True)
p5.setLabel('bottom', 'Time', 's')
p5.setXRange(-10, 0)
pu = pg.PlotDataItem(pen='r')
su = pg.PlotDataItem(pen='b')
legend = p5.addLegend()
legend.addItem(pu, "Primary User")
legend.addItem(su, "Secondary User")
curves = []
data5 = np.empty((chunkSize+1,2))
curves_2 = []
data5_2 = np.empty((chunkSize+1,2))
ptr5 = 0

def update3(newdata1,newdata2):
    global p5, data5, ptr5, curves, data5_2, curves_2
    now = pg.ptime.time()
    for c in curves:
        c.setPos(-(now-startTime), 0)
    
    for c in curves_2:
        c.setPos(-(now-startTime), 0)

    i = ptr5 % chunkSize
    if i == 0:
        curve = p5.plot(pen='r', fillLevel=0, brush=(255, 0, 0, 80))
        curves.append(curve)
        last = data5[-1]
        data5 = np.empty((chunkSize+1,2))        
        data5[0] = last
        while len(curves) > maxChunks:
            c = curves.pop(0)
            p5.removeItem(c)
        
        curve_2 = p5.plot(pen='b', fillLevel=0, brush=(0, 0, 255, 80))
        curves_2.append(curve_2)
        last = data5_2[-1]
        data5_2 = np.empty((chunkSize+1,2))        
        data5_2[0] = last
        while len(curves_2) > maxChunks:
            c = curves_2.pop(0)
            p5.removeItem(c)
    else:
        curve = curves[-1]
        curve_2 = curves_2[-1]
    data5[i+1,0] = now - startTime
    #data5[i+1,1] = data5[i,1]+newdata1
    data5[i+1,1] = newdata1
    curve.setData(x=data5[:i+2, 0], y=data5[:i+2, 1])
    
    data5_2[i+1,0] = now - startTime
    #data5_2[i+1,1] = data5_2[i,1]+newdata2
    data5_2[i+1,1] = newdata2
    curve_2.setData(x=data5_2[:i+2, 0], y=data5_2[:i+2, 1])
    ptr5 += 1


def update4(newdata1):
    global data6,ptr6
    data6[ptr6] = data3[ptr6-1]+newdata1
    ptr6 += 1
    if ptr6 >= data6.shape[0]:
        tmp = data6
        data6 = np.empty(data6.shape[0] * 2)
        data6[:tmp.shape[0]] = tmp
    curve6.setData(data6[:ptr6])

mfilter1 = avg.MovingAverageFilter(2)
mfilter2 = avg.MovingAverageFilter(2)
# update all plots
def update():
#update1()
    global client, pu_bytes, su_bytes, score_su, prev_pu_bytes, prev_su_bytes, otpu, cur_time, prev_time, start_time,stage, prev_thpu, prev_thsu
    #newdata1=np.random.randint(-1,2)
    #newdata2=np.random.randint(-1,2)
    #pu_bytes=np.random.randint(10,100)
    #su_bytes=np.random.randint(10,100)
    #th_pu = float(pu_bytes - prev_pu_bytes )/(50e-3)
    #th_su = float(su_bytes - prev_su_bytes )/(50e-3)
    try:
        th_pu = float(pu_bytes - prev_pu_bytes )/(cur_time-prev_time)
        th_su = float(su_bytes - prev_su_bytes )/(cur_time-prev_time)
    except:
        th_pu = prev_thpu
        th_su = prev_thsu

        pass

    #print th_pu, pu_bytes, prev_pu_bytes
    prev_pu_bytes = pu_bytes
    prev_su_bytes = su_bytes
    otime = cur_time - start_time
    #print "before",th_pu
    filt_th_pu = mfilter1.__call__(th_pu)
    filt_th_su = mfilter2.__call__(th_su)
    full_thpu = pu_bytes/otime
    full_thsu = su_bytes/otime
    spu = np.max([0,(10/9.0)*full_thpu-(otpu/otime/0.987)])
    n_spu = (10/9.0)*full_thpu-(otpu/otime/0.987)
    score = full_thsu * spu 
    score2 = full_thsu * n_spu 
    prev_time = cur_time

    #print "after",th_pu
    #update2(th_pu,th_su)
    #update3(th_pu,th_su)
    if filt_th_pu < 0:
        filt_th_pu = 0
    if filt_th_su < 0:
        filt_th_su = 0
    update2(filt_th_pu,filt_th_su)
    update3(filt_th_pu,filt_th_su)
    #update4(score)
    print "Score:",stage,start_time,score,score2
    #valueChanging(np.random.randint(10))
    prev_thpu = th_pu
    prev_thsu = th_su
sw.Start()
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
