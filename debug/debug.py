import serial 

ser = serial.Serial('COM5', baudrate=9600)

while(1):
    data = ser.read()
    print(data.hex())