##Author: AR; forked from https://github.com/Xuth/tp4000_dmm
##Date Started: 2016/12/27
##Notes: Wireless datalogging for TP4000ZC RS232 multimeters. RS232 protocol available at http://tekpower.us/downloadable/download/sample/sample_id/3/ . Generic manual available at http://tekpower.us/downloadable/download/sample/sample_id/2/ .

from pithy import *
import serial
from pymongo import MongoClient as mc
client = mc("radical-edward.princeton.edu",37017)
import pandas as pd

class Dmm:
    bytesPerRead = 14 #how many bytes are passed in each burst; 4/second
    #the next variable assignments map out how each bit is relevant
    bits = { 
        1: [('flags', 'AC'), ('flags', 'DC'), ('flags', 'AUTO'), ('flags', 'RS232')],
        10:[('scale', 'u'), ('scale', 'n'), ('scale', 'k'), ('measure', 'diode')],
        11:[('scale', 'm'), ('measure', '% (duty-cycle)'), ('scale', 'M'),
            ('flags', 'beep')],
        12:[('measure', 'F'), ('measure', 'Ohms'), ('flags', 'REL delta'),
            ('flags', 'Hold')],
        13:[('measure', 'A'), ('measure', 'V'), ('measure', 'Hertz'),
            ('other', 'other_13_1')],
        14:[('other', 'other_14_4'), ('measure', 'degC'), ('other', 'other_14_2'),
            ('other', 'other_14_1')]}

    digits = [(2,3,'-'), (4,5,'.'), (6,7,'.'), (8,9,'.')]
    digitTable = {(0,5):'1', (5,11):'2', (1,15):'3', (2,7):'4', (3,14):'5',
                  (7,14):'6', (1,5):'7', (7,15):'8', (3,15):'9', (7,13):'0',
                  (6,8):'L', (0,0):' '}

#Ensure proper serial connection here   
    def __init__(self, port='//dev/tty.usbserial-AI0252YA', retries = 3, timeout = 3.0):
        self.ser = serial.Serial(
            port = port,
            baudrate = 2400,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout = timeout)
        self.retries = retries
        self._synchronize()
        
    def _synchronize(self):
        v = self.ser.read(1)
        if len(v) != 1:
            raise DmmNoData()
        n = ord(v)
        pos = n // 16
        if pos == 0 or pos == 15:
            raise DmmInvalidSyncValue()

        bytesNeeded = self.bytesPerRead - pos
        if bytesNeeded:
            v = self.ser.read(bytesNeeded)

    def close(self):
        self.ser.close()

#the next sections read and check for synchronization
    def read(self): 
        success = False
        for readAttempt in xrange(self.retries):
            bytes = self.ser.read(self.bytesPerRead)
            if len(bytes) != self.bytesPerRead:
                self._synchronize()
                continue

            for pos, byte in enumerate(bytes, start=1):
                if ord(byte) // 16 != pos:
                    self._synchronize()
                    break
            else:
                success = True
                break
            self._synchronize()
        if not success:
            raise DmmReadFailure()
        
        val = ''
        for (d1,d2,ch) in self.digits:
            highBit, digit = self._readDigit(bytes[d1-1], bytes[d2-1])
            if highBit:
                val = val + ch
            val = val + digit

        attribs = self._initAttribs()
        for k,v in self.bits.items():
            self._readAttribByte(bytes[k-1], v, attribs)

        return DmmValue(val, attribs, readAttempt, bytes)
                            
    def _initAttribs(self):
        return {'flags':[], 'scale':[], 'measure':[], 'other':[]}

    def _readAttribByte(self, byte, bits, attribs):
        b = ord(byte) % 16
        bitVal = 8
        for (attr, val) in bits:
            v = b // bitVal
            if v:
                b = b - bitVal
                #print "adding flag type %s, val %s"%(attr, val)
                attribs[attr].append(val)
            bitVal //= 2

#this function matches bytes with value from digitTable
    def _readDigit(self, byte1, byte2): 
        b1 = ord(byte1) % 16
        highBit = b1 // 8
        b1 = b1 % 8
        b2 = ord(byte2) % 16
        try:
            digit = self.digitTable[(b1,b2)]
        except:
            digit = 'X'
        return highBit, digit
            

class DmmValue:

    scaleTable = {'n': 0.000000001, 'u': 0.000001, 'm': 0.001, 'k': 1000.0, 'M': 1000000.0}

#this function takes  received data and assigns attributes
    def __init__(self, val, attribs, readErrors, rawBytes):
        self.saneValue = True
        self.rawVal = self.val = val
        self.flags = attribs['flags']
        self.scaleFlags = attribs['scale']
        self.measurementFlags = attribs['measure']
        self.reservedFlags = attribs['other']
        self.readErrors = readErrors
        self.rawBytes = rawBytes
        self.text = 'Invalid Value'

        self.processFlags()
        self.processScale()
        self.processMeasurement()
        self.processVal()

        if self.saneValue:
            self.createTextExpression()

#this function assigns the output mode to ACDCText
    def processFlags(self):
        flags = self.flags
        self.ACDC = None
        self.ACDCText = ''
        self.delta = False
        self.deltaText = ''

        if 'AC' in flags and 'DC' in flags:
            self.saneValue = False
        if 'AC' in flags:
            self.ACDC = 'AC'
        if 'DC' in flags:
            self.ACDC = 'DC'
        if self.ACDC is not None:
            self.ACDCText = ' ' + self.ACDC
        if 'REL delta' in flags:
            self.delta = True
            self.deltaText = 'delta '

#this function takes scale and sets up the corresponding value multiplier
    def processScale(self):
        s = self.scaleFlags
        self.scale = ''
        self.multiplier = 1

        if len(s) == 0:
            return
        if len(s) > 1:
            self.saneValue = False
            return
        self.scale = s[0]
        self.multiplier = self.scaleTable[self.scale]

#this function determines which measurement is being taken
    def processMeasurement(self):
        m = self.measurementFlags
        self.measurement = None
        if len(m) != 1:
            self.saneValue = False
            return
        self.measurement = m[0]

#this function scales value to appropriate magnitude and properly formats
    def processVal(self): 
        v = self.rawVal
        self.numericVal = None
        if 'X' in v:
            self.saneValue = False
            return
        if v.count('.') > 1:
            self.saneValue = False
            return

        n = None
        try:
            n = float(v)
        except:
            pass

        if n is not None:
            self.val = '%s'%n
            self.numericVal = n * self.multiplier

#this function concatenates the entire text string    
    def createTextExpression(self):
        text =  self.deltaText
        text += self.val
        text += ' '
        text += self.scale
        text += self.measurement
        text += self.ACDCText
        self.text = text
        
    def __repr__(self):
        return "<DmmValue instance: %s>"%self.text

class DmmException:
    "Base exception class for Dmm."

class DmmNoData(DmmException):
    "Read from serial port timed out with no bytes read."

class DmmInvalidSyncValue(DmmException):
    "Got an invalid byte during synchronization."

class DmmReadFailure(DmmException):
    "Unable to get a successful read within number of allowed retries."

def dataoutput():
    dmm = Dmm()
    info = {}
    infolist = []
    
    while True: 
        try:
            val = dmm.read()
            info['time'] = time.time()
            info['value'] = val.numericVal #scaled, base SI unit
            info['scale'] = val.scale #scale
            info['unit'] = val.measurement #unit
            info['mode'] = val.ACDCText #AC vs. DC
            info['dmm'] = 'DMM1' #update identifier later wrt interface
            infolist.append(info.copy())
            # print val.val #outputs value displayed on dmm screen.
            # print val.text #outputs everything formatted as a str
        except DmmNoData:
            break
    
    for i in range(0,len(infolist)-1):
        client.mmdb.col4.insert(infolist[i])
    
    client.mmdb.col4.create_index('time')
    c = client.mmdb.col4.find({}).sort('time',1)
    df = pd.DataFrame(list(c))
    print df
    plot(df['time'],df['value']) 
    xlabel('Time')
    ylabel(df['unit'][5])
    showme(dpi=100)
    clf()

if __name__ == "__main__":
    dataoutput()
