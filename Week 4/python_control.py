import serial

ser = serial.Serial('/dev/ttyACM0')  # open serial port                         
print(ser.name)         # check which port was really used                      
ser.write(b'MOVEF:1000\n')     # write a string                                 
ser.write(b'MOVEB:1000\n')     # write a string                                 
ser.close()             # close port
