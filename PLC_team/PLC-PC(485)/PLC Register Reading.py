import minimalmodbus as minimalmodbus
import serial

if __name__ == '__main__':
    instrument = minimalmodbus.Instrument("COM4", 1,'rtu')
    instrument.serial.baudrate = 9600  # Baud
    instrument.serial.bytesize = 8
    instrument.serial.parity = serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout = 1  # seconds
    
    for i in range(1, 100):
        try:
            print(instrument.read_register(i, functioncode=int('0x03', 16)))
        except IOError:
            print("Failed to read from instrument")