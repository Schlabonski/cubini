import time
import numpy as np
import usb.core as usb
from .vcp_terminal import ComPort
from struct import pack

class KPZ101():
    channel = 1
    destination = 0x50
    source = 0x01
    hub_analog_IP = 0x01 # only matters in case of closed loop operation
    
    
    def __init__(self, serial_number=None):    
        self.com = None
        
        # find matching USB devices
        devs = usb.find(find_all=True, idVendor=0x0403, idProduct=0xfaf0)
        kpz = None
        for dev in devs:
            try:
                sn = usb.util.get_string(dev, 3, dev.iSerialNumber)
                if sn == str(serial_number):
                    kpz = dev
                    #print(kpz)
            except:
                pass
            
        assert kpz is not None, 'No KPZ101 with matching serial number {} found!'.format(serial_number)
        
        time.sleep(.1)
        # open serial communication channel with the device
        com = ComPort(usb_device=kpz)
        
        # initialize FTDI chip according to APT documentation
        com.setLineCoding(baudrate=115200, databits=8, stopbits=1)
        
        # Get HW info; MGMSG_HW_REQ_INFO; may be require by a K Cube to allow confirmation Rx messages
        # use the length of the response as a check for uncorrupted communication
        com.write(pack('<HBBBB', 0x0005, 0x00, 0x00, 0x50, 0x01))
        time.sleep(0.1)
        hw_info = com.readBytes()
        assert len(hw_info) == 90, 'Communication corrupted for KPZ101 SN {}, response length {} != 90 bytes.'.format(serial_number, len(hw_info))

        self.com = com
        time.sleep(0.1)
    
    
    def __del__(self):
        if self.com is not None:
            self.com.disconnect()

        
    def set_max_voltage(self, max_voltage=75):
        '''
        Sets the maximum output voltage and update the voltage scale factor.
        MGMSG_PZ_SET_TPZ_IOSETTINGS
        '''
        voltage_limit_byte = {'75':0x01, '100':0x02, '150':0x03}
        assert str(max_voltage) in voltage_limit_byte.keys(), 'Invalid output voltage limit!'
        
        self.max_voltage = max_voltage
        self.device_unit_sf = int(32767./max_voltage)
        vlb = voltage_limit_byte[str(max_voltage)]
        
        self.com.write(pack('<HBBBBHHHHH',0x07D4,0x0A,0x00,self.destination|0x80,self.source,self.channel,vlb,self.hub_analog_IP,0x00,0x00))
    
    
    def set_input_mode(self, input_mode='software'):
        '''
        Sets the input mode. For documentation check APT docs p. 160.
        MGMSG_PZ_SET_INPUTVOLTSSRC
        '''
        
        assert input_mode in ['software', 'external', 'potentiometer'], 'Invalid input type!'
        input_mode_byte_dict = {'software':0x00, 'external':0x01, 'potentiometer':0x02}
        input_byte = input_mode_byte_dict[input_mode]
        
        self.com.write(pack('<HBBBBHH',0x0652,0x04,0x00,self.destination|0x80,self.source,self.channel,input_byte))
        
    
    def enable_output(self):
        '''
        Enables the high voltage ouput.
        MGMSG_MOD_SET_CHANENABLESTATE
        '''
        self.com.write(pack('<HBBBB',0x0210,self.channel,0x01,self.destination,self.source))

    
    def disable_output(self):
        '''
        Enables the high voltage ouput.
        MGMSG_MOD_SET_CHANENABLESTATE
        '''
        self.com.write(pack('<HBBBB',0x0210,self.channel,0x02,self.destination,self.source))
    
    def set_output_voltage(self, v=0):
        '''
        Set the output voltage.
        MGMSG_PZ_SET_OUTPUTVOLTS
        '''
        assert v>=0 and v<=self.max_voltage, 'Voltage out of limits!'
        voltage_device_units = int(self.device_unit_sf * v)
        self.com.write(pack('<HBBBBHH',0x0643,0x04,0x00,self.destination|0x80,self.source,self.channel,voltage_device_units))
