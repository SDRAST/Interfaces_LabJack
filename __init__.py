# -*- coding: utf-8 -*-
"""
Monitor and Control  with LabJack

For details see the LabJack U3 User Manual::
 http://labjack.com/support/u3/users-guide and
 https://github.jpl.nasa.gov/pages/RadioAstronomy/Electronics/

LabJack
=======
Each U3 has four FIO sections with these pins::
  VS     VS     VS     VS    - 5 V supply
  GND    GND    GND    GND   - ground
  FIO0   FIO2   FIO4   FIO6  - FIO
  FIO1   FIO3   FIO5   FIO7  - FIO
The fifth and sixth blocks are::
  VS    VS
  SGND  GND
  SPC   DAC0
  SGND  DAC1
FIO pins can be::
  analog input     (FIOAnalog = 1, FIOBitDir = 0)
  digital input    (FIOAnalog = 0, FIOBitDir = 0)
  digital output   (FIOAnalog = 0, FITBitDir = 1)
  
In addition, the DB15 connector has twelve digital-only lines,
EIO0-EIO7 and CIO0 - CIO3.

An initial checkout might be::
  In [2]: from Electronics.Interfaces.LabJack import *
  In [3]: lj = connect_to_U3s()

The normal state of the U3s is something like::
          U3 local ID          1        2
          ------------- -------- --------
              CIOBitDir 00001111 00000000
               CIOState 00001111 00001111
             DAC1Enable 00000000 00000000
              EIOAnalog 00000000 00000000
              EIOBitDir 11111111 00000000
               EIOState 01010111 11111111
         EnableCounter0    0.000    0.000
         EnableCounter1    0.000    0.000
                  FAIN0    0.001
                  FAIN1    1.455
                  FAIN2    0.000
                  FAIN3    0.000
              FIOAnalog 00001111 00000000
              FIOBitDir 00000000 00000000
               FIOState 11110000 11111111
  NumberOfTimersEnabled        0        0
          Temperature  295.569  295.507
     TimerCounterConfig       64       64
  TimerCounterPinOffset        4        4

Classes Summary
===============
These are the classes in this module and their attributes and methods::
  class LabJack():
    get_AINs(prefix)
  class LJTickDAC():
    __init__(self, device, IO_chan=0)
    report_LabTick(self)
    setVoltages(self,voltages)
    getCalConstants(self)
  class AIN_Reader(Thread)
    __init__(self, device, pinNum, readInterval):
    stop(self)
    run(self)

Module Functions
================
These are the functions in this module.

General Functions
-----------------
These functions are for general support::
  toDouble(buffer)
  searchForDevices()
  connect_to_U3s()
  close_U3s(lj)

LabJack Functions
-----------------
These functions deal with LabJack configuration::
  get_U3s_config(lj)
  get_IO_states(lj)
  report_U3_config(config)
  report_IO_config(config)
"""
import sys
import re
import u3
from LabJackPython import listAll, openAllLabJacks
import time
import struct
import numpy as NP
from threading import Thread
import logging

import Math
from support import unique
from Electronics.Instruments import VoltageSource

module_logger = logging.getLogger(__name__)

U3name = {}

LJchan = {"FIO0": 0, "FIO1": 1, "FIO2": 2, "FIO3": 3,
          "FIO4": 4, "FIO5": 5, "FIO6": 6, "FIO7": 7,
          "EIO0": 8, "EIO1": 9, "EIO2":10, "EIO3":11,
          "EIO4":12, "EIO5":13, "EIO6":14, "EIO7":15,
          "CIO0":16, "CIO1":17, "CIO2":18, "CIO3":19}

################################# functions ###################################

def toDouble(buffer):
  """
  toDouble - Converts the 8 byte array into a floating point number.
  
  Used to convert I2C calibration bytes to I2C constants

  The slopes and offsets are stored in 64-bit fixed point format (signed
  32.32, little endian, 2’s complement).

  @type buffer : bit string
  @param buffer : an array with 8 bytes

  @return: float
  """
  if type(buffer) == type(''):
    bufferStr = buffer[:8]
  else:
    bufferStr = ''.join(chr(x) for x in buffer[:8])
  dec, wh = struct.unpack('<Ii', bufferStr)
  return float(wh) + float(dec)/2**32
  
def searchForDevices():
    """
    Determines if U3 devices are available
    """
    u3Available = listAll(3) # argument must be 3 for U3
    return u3Available.keys()

def connect_to_U3s(localIDs=None):
  """
  Open all the U3s

  Return a dictionary of LabJack instances indexed by local ID.  This is
  different from u3.openAllU3() which returns a list of U3 instances
  indexed by serial number.

  If dict localIDs is not given, the localID already in the LabJack is used.

  @param localIDs : optional {serialNo:localID} assigned to the LabJacks
  @type  localIDs : dict

  @return: dict of LabJack instances
  """
  lj = {}
  try:
    U3s = listAll(3) # openAllU3()
  except Exception, details:
    print "Could not list all U3s"
    print details
    return {}
  if localIDs == None:
    serials = U3s.keys()
  else:
    serials = localIDs.keys()
  for serialno in serials :
    if localIDs == None:
      localID = U3s[serialno]['localId']
    else:
      localID = localIDs[serialno]
    lj[localID] = LabJack(serialno)
  if len(lj) == 0:
    print "No LabJack U3s found.  Is USB connected?"
  return lj

def report_U3_config(lj):
  """
  Prints the U3 power-up configuration of all U3s

  @type config : dictionary
  @param config : result from get_IO_states()

  @return: None
  """
  response = ""
  U3s = lj.keys()
  pars = ['SerialNumber', 'LocalID', 'DeviceName',
          'HardwareVersion', 'FirmwareVersion', 'BootloaderVersion',
          'FIOAnalog', 'FIODirection', 'FIOState',
          'EIOAnalog', 'EIODirection', 'EIOState',
          'CIODirection', 'CIOState']
  response += "========LJ U3 power-up configurations========\n"
  response += "U3 ID:           "
  for ID in U3s:
    response += ("%10d" % ID)
  response += "\n"
  for par in pars:
    response += ("%21s" % par)
    for ID in U3s:
      value = lj[ID].u3_config[par]
      if re.search('IO',par):
        response += ("%10s" % Math.decimal_to_binary(value,8))
      elif type(value) == float:
        response += ("%10.3f" % value)
      elif type(value) == int:
        response += ("%10d" % value)
      else:
        response += ("%10s" % value)
    response += "\n"
  return response
    
def report_IO_config(lj):
  """
  Print the IO state of all U3s

  @type config : dictionary
  @param config : result from get_IO_states()

  @return: None
  """
  U3s = lj.keys() # list of U3s
  par_keys = []
  for ID in U3s:
    par_keys += lj[ID].IO_config.keys()
  params = unique(par_keys)
  params.sort()
  response = "=============== U3 I/O Configurations ================\n"
  response += "U3 local ID:           "
  for U3 in U3s:
    response += ("%4d     " % U3)
  response += "\n"
  for param in params:
    response += ("%22s " % param)
    for ID in U3s:
      if lj[ID].IO_config.has_key(param):
        value = lj[ID].IO_config[param]
        if re.search("Timer",param):
          response += ("%8d " % value)
        elif type(value) == int:
          response += Math.decimal_to_binary(value,8)+" "
        elif type(value) == float:
          response += ("%8.3f " % value)
        else:
          response += "         "
      else:
        response += "         "
    response += "\n"
  return response
      
def close_U3s(lj):
  """
  Close all U3s

  @type lj : dictionary of u3.U3 class instances keyed by local ID
  @param lj : LabJack U3() instances to be closed
  """
  for ID in lj.keys():
    lj[ID].close()

################################# LabJack classes #############################

class LabJack(u3.U3):
  """
  U3 subclass with additional attributes and methods.

  Some inherited methods::
    binaryToCalibratedAnalogTemperature()
    binaryToCalibratedAnalogVoltage() Bits returned from AIN into voltage.
    close()
    configAnalog()      same as configIO()
    configDigital()     same as configIO()
    configIO()          reports DAC1Enable, EIOAnalog, DIOAnalog and timers
    configTimerClock()
    configU3()          report or set configuration parameters
    getAIN()
    getDIOState()       read the state of a digital I/O channel
    getDIState()        A convenience function to read the state of an FIO.
    getFeedback()       Sends a commandlist to the U3, and reads the response.
    getName()
    getTemperature()    Reads the internal temperature sensor on the U3.
    loadConfig()        Takes a configuration and updates the U3 to match it.
    reset()             Causes a soft or hard reset.
    setDIOState()       same as setDOState
    setDOState()        Set the state of a digital I/O channel
    setName()
    setToFactoryDefaults()
    toggleLED()         Toggles the state LED on and off.
    voltageToDACBits()  Takes a voltage, and turns it into the bits
    watchdog()          read/write the configuration of the watchdog

  Some inherited attributes::
    name
    
  """
  def __init__(self,serialno):
    """
    Instantiate a LabJack

    If the Labjack is known by it serial number then we know where it is and
    can make sure that it has the right local ID.  The set below are for the
    Labjacks in WBDC1.  For other Labjacks we trust that the local ID was set
    correctly in the configuration.

    @param serialno : serial number of the LabJack
    @type  serialno : str
    """
    mylogger = logging.getLogger(module_logger.name+".LabJack")
    u3.U3.__init__(self,autoOpen = True, serial = int(serialno))
    self.logger = mylogger
    self.u3_config = self.configU3()
    self.IO_config = self.configIO()
    self.IO_state = self.get_IO_states()
    
    self.serial = self.u3_config['SerialNumber']
    self.localID = self.u3_config['LocalID']
    self.logger = logging.getLogger(__name__+".LabJack")

  def get_AINs(self,prefix):
    """
    Get values for all defined analog inputs

    @type prefix : str
    @param prefix : "F" or "E"

    @return: dictionary keyed by signal names
    """
    values = {}
    try:
      response = self.configIO()
    except Exception, details:
      print "Could not get IO configuration for LabJack",self.localID
      return {}
    else:
      if prefix.upper() == 'E':
        AINbits = response["EIOAnalog"]
      elif prefix.upper() == 'F':
        AINbits = response["FIOAnalog"]
      else:
        return values
      for bit in range(8):
        bitvalue = 2**bit
        if prefix.upper() == 'E':
          bitID = bit + 8
        else:
          bitID = bit
        if bitvalue & AINbits:
          values[prefix.upper()+'AIN'+str(bit)] = self.getFeedback(u3.AIN(bitID))
      return values

  def get_dir_bits(self):
    """
    Get direction bits
    """
    response = self.configU3()
    self.logger.debug("%s", response)
    return response["FIODirection"], \
           response["EIODirection"], \
           response["CIODirection"]

  def configureIO(self, config_dict):
    """
    Configure the IO ports

    This sets the IO onfiguration parameters::
      'FIODirection', 'FIOAnalog',
      'EIODirection', 'EIOAnalog',
      'CIODirection'

    Example::
      config_dict = {'FIODirection': 0,
                     'FIOAnalog':    int('00000011',2),
                     'EIODirection': int('11111111',2),
                     'EIOAnalog':    0,
                     'CIODirection': 0}

    In fact, this is not limited to these keys but can be used with any valid
    configuration keys.

    @param config_dict : dict with above keys and byte values
    @type  config_dict : {str:int, ...}
    """
    command = ""
    for key in config_dict.keys():
      print "Processing",key
      command += key+"="+str(config_dict[key])+","
      print command
    eval( "self.configIO(" + (command[:-1]) + ")" )
      
  def set_DO_bits(self, prefix, byte):
    """
    Set digital output bits

    @type prefix : str
    @param prefix : "F", "E", or "C" to designate the port

    @type byte : int
    @param byte : value to write to port
    """
    Fdir,Edir,Cdir = self.get_dir_bits()
    self.logger.debug("For %s, direction is %d, %d, %d", prefix,Fdir,Edir,Cdir)
    if prefix.upper() == "F":
      mask = [Fdir,0,0]
      state = [byte,0,0]
      self.logger.debug("Mask %s, State %s",
                        Math.decimal_to_binary(mask[0],8),
                        Math.decimal_to_binary(state[0],8))
    elif prefix.upper() == "E":
      mask = Edir
      state = [0,byte,0]
    elif prefix.upper() == "C":
      mask = Cdir
      state = [0,0,byte]
    self.getFeedback(u3.PortStateWrite(State=state, WriteMask=mask))
    
  def pulse_bit(self,bit):
    """
    This drops the designated bit to state 0 for half a second

    To ensure that it is seen as a negative pulse, the bit is
    set high for one second if it was not already high.
    """
    if self.getFeedback(u3.BitStateRead(IONumber = bit)) == 0:
      self.getFeedback(u3.BitStateWrite(IONumber = bit, State = 1))
      time.sleep(1)
    self.getFeedback(u3.BitStateWrite(IONumber = bit, State = 0))
    time.sleep(0.5)
    self.getFeedback(u3.BitStateWrite(IONumber = bit, State = 1))

  def get_IO_states(self):
    """
    Get the I/O state of the connected U3s

    @type lj : dictionary
    @param lj : u3.U3 class instances keyed by local ID

    @return: dictionary
    """
    self.IO_state = {}
    response = {}
    EAINvalues = self.get_AINs("E")
    for AIN in EAINvalues.keys():
      response[AIN] = self.binaryToCalibratedAnalogVoltage(EAINvalues[AIN][0],
                                                           isLowVoltage=True,
                                                           isSingleEnded=True)
    FAINvalues = self.get_AINs("F")
    for AIN in FAINvalues.keys():
      response[AIN] = self.binaryToCalibratedAnalogVoltage(FAINvalues[AIN][0],
                                                           isLowVoltage=True,
                                                           isSingleEnded=True)
    try:
      bitdirs = self.getFeedback(u3.PortDirRead())[0]
    except Exception,details:
      self.logger.error("Could not get direction bits for LabJack %d",
                        self.localID)
    else:
      response["CIOBitDir"] = bitdirs["CIO"]
      response["EIOBitDir"] = bitdirs["EIO"]
      response["FIOBitDir"] = bitdirs["FIO"]
    try:
      bitstates = self.getFeedback(u3.PortStateRead())[0]
    except Exception,details:
      self.logger.error("Could not get states of bits for LabJack %s",
                        self.localID)
    else:
      response["CIOState"] = bitstates["CIO"]
      response["EIOState"] = bitstates["EIO"]
      response["FIOState"] = bitstates["FIO"]
      response["Temperature"] = self.getTemperature()
    self.IO_state = response
    return self.IO_state
  
class LJTickDAC(object):
  """
  Each FIO section, when configured for digital output, can accomodate a
  TickDAC with two ADCs in the range of -10 to +10 V.  Pins VS and GND power
  the TickDAC. The two IO ports in each block (e.g. FIO(x) and FIO(x+1))
  are used to communicate with the TickDAC using Inter-Integrated Circuit (I2C)
  protocol. A specific TickDAC is selected using the LabJack channel numbers
  (see LJchan above).  The even-numbered channel provides CLK and odd-numbered
  channel provides SDA.

  The TickDAC has a four screw connector whose pins are::
    VS    - 4 V supply
    GND   - ground
    DACA  - +/- 10 V with 14-bit resolution
    DACB  - +/- 10 V with 14-bit resolution

  The TickDAC has a non-volatile 128-byte EEPROM on the I2C bus with a
  7-bit address of 0x50 (decimal 80). Bytes 0-63 are available to the user,
  while bytes 64-127 are reserved::
    EEPROM Address  Description   Nominal Value
     0- 63         User Area
    64- 71         DACA Slope    3.1586E+03 bits/volt
    72- 79         DACA Offset   3.2624E+04 bits
    80- 87         DACB Slope    3.1586E+03 bits/volt
    88- 95         DACB Offset   3.2624E+04 bits
    96- 99         Serial Number
   100-127         Reserved
  The slopes and offsets are stored in 64-bit fixed point format
  (signed 32.32, little endian, 2's complement). The serial number is
  simply an unsigned 32-bit value where byte 96 is the LSB and byte 99
  is the MSB.

  The DAC (digital-to-analog converter) chip on the TickDAC has a 7-bit
  address of 0x12 (d18). The data is justified to 16 bits, so a binary
  value of 0 (actually 0-3) results in minimum output (~-10.3 volts) and
  a binary value of 65535 (actually 65532-65535) results in maximum
  output (~10.4 volts).

  Notes
  =====
  
  This version for LabJack U3 only.

  In LJTickDAC demo setup, the DAC Pins options are FIO 4/5 and FIO6/7
  The first associates self.dacPin with a value of 0.
  The second associates self.dacPin with a value of 2.
  The AIN Pins options are None, AIN/FIO 0, and AIN/FIO 2.
  The first associates self.ainPin with a value of 0.
  The second associates self.ainPin with a value of 2
  It says that the AIN pins are provided for testing.
  There are entry fields for DAC A and DAC B.
  """
  U3 = 3
  DAC_PIN_DEFAULT = 0
  EEPROM_address = 0x50
  DAC_address = 0x12
  
  def __init__(self, labjack, name, IO_chan=0):
    """
    @param device : LabJack with attached TickDAC
    @type devicec : u3.U3 class instance

    @param name : identifier for this TickDAC
    @type  name : str
    
    @param IO_chan : lower of FIO pin pair: 0, 2, 4, or 6
    @type IO_chan : int
    """
    self.logger = logging.getLogger(module_logger.name+".LJTickDAC")
    self.parent = labjack
    self.name = name
    self.dacPin = IO_chan
    self.data = {'A': self.Channel(self,'A'),
                 'B': self.Channel(self,'B')}
    self.getCalConstants()
    self.voltages = [None,None]

  def getCalConstants(self):
    """
    Loads or reloads the calibration constants for the LJTic-DAC:
    self.aSlope
    self.aOffset
    self.bSlope
    self.bOffset

    Notes
    =====
    This is for U3 only

    The LJTDAC has a non-volatile 128-byte EEPROM (Microchip 24C01C) on the I2C
    bus with a 7-bit address of 0x50 (d80), and thus an 8-bit address byte of
    0xA0 (d160). Bytes 0-63 are available to the user, while bytes 64-127 are
    reserved::
      EEPROM
      Address                   Nominal Value
       0- 63  User Area
      64- 71  DACA Slope        3.1586E+03  bits/volt
      72- 79  DACA Offset       3.2624E+04  bits
      80- 87  DACB Slope        3.1586E+03  bits/volt
      88- 95  DACB Offset       3.2624E+04  bits
      96- 99  Serial Number
     100-127  Reserved

    The DAC (digital-to-analog converter) chip on the LJTDAC is the LTC2617
    (linear.com) with a 7-bit address of 0x12 (d18), and thus an 8-bit address
    byte of 0x24 (d36). The data is justified to 16 bits, so a binary value of
    0 (actually 0-3) results in minimum output (~-10.3 volts) and a binary
    value of 65535 (actually 65532-65535) results in maximum output
    (~10.4 volts).

    The serial number is simply an unsigned 32-bit value where byte 96 is the
    LSB and byte 99 is the MSB.
    """
    # Determine pin numbers
    sclPin = self.dacPin
    sdaPin = sclPin + 1

    # Make request
    data = self.parent.i2c(LJTickDAC.EEPROM_address,
                           [64],
                           NumI2CBytesToReceive=36,
                           SDAPinNum = sdaPin,
                           SCLPinNum = sclPin)

    self.logger.debug("getCalConstants: data keys: %s",
                      data.keys())
    response = data['I2CBytes']
    self.logger.debug("getCalConstants: response = %s",response)
    self.data['A'].slope = toDouble(response[0:8])
    self.data['A'].offset = toDouble(response[8:16])
    self.data['B'].slope = toDouble(response[16:24])
    self.data['B'].offset = toDouble(response[24:32])
    self.serial = struct.unpack('<i',
                                ''.join(chr(x) for x in response[32:36]))[0]

    if 255 in response:
      self.logger.error("getCalConstants: %s", response)
      self.logger.warning(" Please go into settings\n"+
           "  and make sure the pin numbers are correct\n"+
           "  and that the LJTickDAC is properly attached.")

  def report(self):
    """
    Report on the attributes of the LabTick instance
    """
    report = []
    report.append("Serial number:     "+str(self.serial))
    report.append("FIO base channel:  "+str(self.dacPin))
    report.append("Slopes:  %7.1f  %7.1f" % (self.data['A'].slope,
                                             self.data['B'].slope))
    report.append("Offsets: %7.1f  %7.1f" % (self.data['A'].offset,
                                             self.data['B'].offset))
    return report

  def __str__(self):
    return self.base()+' "'+self.name+'"'

  def __repr__(self):
    return self.base()+' "'+self.name+'"'

  def base(self):
    """
    String representing the class instance type
    """
    return str(type(self)).split()[-1].strip('>').strip("'").split('.')[-1]

  def __setitem__(self, key, item):
    self.data[key] = item

  def __getitem__(self, key):
    return self.data[key]

  def keys(self):
    return self.data.keys()

  def has_key(self,key):
    if self.data.has_key(key):
      return True
    else:
      return False

  class Channel(VoltageSource):
    """
    One of the LJTickDAC output channels

    Attributes and methods inherited from Voltage Source::
      logger        - logging.Logger instance
      volts         - current voltage value
      get_voltage() - asks for current voltage
      set_voltage() - sets output voltage
    """
    def __init__(self, parent, name):
      """
      @param parent : LJTickDAC to which this belongs
      @type  parent : LJTickDAC instance
      
      @param name : "A" or "B", lowercase allowed
      @type  name : str
      """
      self.parent = parent
      self.name = name
      mylogger = logging.getLogger(self.parent.name+".Channel")
      mylogger.debug(" initialized %s channel %s", self, name)
      name = name.upper()
      if name == 'A' or name == 'B':
        self.name = name
      else:
        mylogger.error("invalid name %s", name)
        raise ObservatoryError("invalid name")
      VoltageSource.__init__(self, name, parent=parent)
      self.logger = mylogger
      self.index = ord(name) - ord('A')
      self.volts = None

    def setVoltage(self, voltage):
      """
      """
      self.logger.debug("setVoltage: setting %s to %f V", self, voltage)
      self.logger.debug("setVoltage: LabJack is %s", self.parent.parent)
      try:
        self.parent.parent.i2c(LJTickDAC.DAC_address,
                               [48+self.index,
                                int(((voltage*self.slope) + self.offset)/256),
                                int(((voltage*self.slope) + self.offset)%256)],
                               SDAPinNum = self.parent.dacPin+1,
                               SCLPinNum = self.parent.dacPin)
        self.volts = voltage
        return True
      except Exception, details:
        self.logger.error("setVoltage: failed %s", sys.exc_info())
        return False
      

class AIN_Reader(Thread):
  """
  A thread that reads from a specified analog input every interval

  I think this can re replaced by DeviceReadThread

  Public Attributes
  =================
  device   - LabJack U3() instance
  pinNum   - pin number of analog input
  interval - time in seconds (float) between readings
  running  - thread state
  voltage  - reading in V
  
  Notes
  =====
  For U3 only
  """
  def __init__(self, device, pinNum, readInterval):
    """
    Creates a thread and associates it with a specific LabJack device
    and pin number on that device.

    @type device : u3.U3 instance
    @param device : LabJack with designated pin

    @type pinNum : int
    @param pinNum : pin to be read

    @type readInterval : float
    @param readInterval : time in sec between readings
    
    @return: None
    """
    Thread.__init__(self)
    self.device = device
    self.pinNum = pinNum
    self.interval = readInterval
  
  def stop(self):
    """
    Stops this thread
    """
    self.running = False
  
  def run(self):
    """
    Starts this thread
    """
    try:
      self.running = True
      while self.running:
        self.voltage = self.device.getAIN(self.pinNum)
        time.sleep(self.interval)
    except:
      print "AIN read error. Device detached?"
