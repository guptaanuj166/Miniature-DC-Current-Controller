import serial
import time
import streamlit as st
import csv
import serial.tools.list_ports
# print(list())

st.title("Portable current controller-cum-indicator for panel mounting")
st.subheader("Group TUE-04, Project P-09")

st.write("Harish Mokashi, Mayank Gupta, Anuj Gupta, Vansh Aggarwal")

start = st.checkbox('Connect')
if start:
    # ser = serial.Serial(port=str(serial.tools.list_ports.comports()[0]),baudrate=115200)
    ser = serial.Serial(port='COM3',baudrate=115200)

    data1='re'
    data2='ad'
    data_eol='\r'

    data4='wr'
    data5='it'
    data6='e\r'

    data7='st'
    data8='op'
    data9=' '
    
    read = st.checkbox('Read')
    f_read=1
    if read and f_read:
        f_read=0
        ser.write(data1.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data2.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data_eol.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"))
    else:
        f_read=1
        
        ser.write(data7.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data8.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data9.encode())
        
        ser.write(data1.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data2.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data_eol.encode())
        
    if read:
        st.write("Actual current (in A)")
        t= st.empty()
        # while True:
        cc=ser.read(8)
        unit=cc.decode("utf-8")
        t.write(unit[:-2]+" A")
            # st.rerun()
            


        

    write = st.checkbox('Write')
    f_write=1
    if write and f_write:
        f_write=0
        ser.write(data4.encode())
        ser.write(data5.encode())
        ser.write(data6.encode())
        # ser.write(data7.encode())
        
    else:
        f_write=1
        ser.write(data7.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data8.encode())
        # cc=ser.read(12)
        # print(cc.decode("utf-8"), end='')
        ser.write(data9.encode())
        
        ser.write(data4.encode())
        ser.write(data5.encode())
        ser.write(data6.encode())
        
    p_write=0.00

    if write:
        # while True:
        set_current=st.slider("Set Current: ", value=0.00, min_value=0.00, max_value=3.00, step = 0.01)
    
    
        if (set_current>=0 and set_current<=3 and set_current!=p_write):
            p_write=set_current
            a=str(set_current)
            a=a[:2]
            b=str(set_current)
            b=b[2:]
            if len(b)==1:
                b+='0'
            
            ser.write(data_eol.encode())
            ser.write(a.encode())
            ser.write(b.encode())
            ser.write(data_eol.encode())
        
    # tri = st.checkbox('Triangle')
        
    # p_write_tri=0.00

    # if tri:
        
        # set_current_tri=st.slider("Set Peak Current: ", value=0.00, min_value=0.00, max_value=3.00, step = 0.01)
        # set_freq=st.slider("Set Peak Current: ", value=0.01, min_value=0.0, max_value=10.0, step = 0.01)
        
        # start=0.0
        # if (set_current_tri>=0 and set_current_tri<=3 and set_current_tri!=p_write_tri):
            # p_write_tri=set_current_tri
            # for i in range(int(set_current_tri*50)):
                # a=str(start)
                # a=a[:2]
                
                # b=str(start)
                # b=b[2:]
                # if len(b)==1:
                    # b+='0'
                
                # start+=0.02
                # ser.write(data_eol.encode())
                # ser.write(a.encode())
                # ser.write(b.encode())
                # ser.write(data_eol.encode())
                # time.sleep(1/2/set_freq/int(set_current_tri*50))
    bt=st.button("Update")   
            
    # with open('current.csv', 'w', newline=' ') as csvfile:
            # writer = csv.writer(csvfile)
            # writer.writerow(['Current (in A)']) 

    # while True:
        # ser.write(data2.encode())

        # cc=ser.read(12)
        # print(cc.decode("utf-8"))
        # writer.writerow([cc.decode("utf-8")])
    # Close the serial port

    # stop = st.checkbox('Stop')
    # if stop:
        # ser.close()