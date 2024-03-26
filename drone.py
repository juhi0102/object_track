import socket #To form the connection between the drone and the laptop
import time
import math
    
MSP_STATUS=101          # out cmd cycletime & errors_count & sensor present & box activation & current setting number
MSP_RAW_IMU=102         # 9 DOF 
MSP_ATTITUDE=108        # 2 angles 1 heading
MSP_ALTITUDE=109        # altitude, variometer
MSP_ANALOG=110          # vbat, powermetersum, rssi if available on RX
MSP_SET_RAW_RC=200      # 8 rc channel
MSP_SET_COMMAND=217     # setting commands 
RETRY_COUNT=3           # no of retries before getting required data

MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206

class drone:
    #def __init__(self, DroneIP="192.168.0.1", DronePort=9060): #default pluto settings
    def __init__(self, DroneIP="192.168.4.1", DronePort=23): #default pluto settings
        self.DRONEIP = DroneIP
        self.DRONEPORT = DronePort
        self.BUFFER_SIZE = 4096
        #Initailizing the values
        self.roll=1500                    
        self.pitch=1500                 
        self.throttle=1500 
        self.yaw=1500                      
        self.aux1=1200
        self.aux2=1000
        self.aux3=1500
        self.aux4=1200
        
        self.buffer_rc=bytearray([])               # rc data that has to be sent continuously 
        self.trim(0,0,0,0) #To stabalize the drone. Initally the trim values are set to 0 and can be changed according to the drift of the drone

    def connect(self):
        '''
        Function to form the connection with thr drone
        '''
        try:
            self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.mySocket.connect((self.DRONEIP, self.DRONEPORT))
            print("pluto connected")
            self.get_battery()
        except:
            
            print("Cannot connect to pluto, please try again...")

    def disconnect(self):
        '''
        Function to close the connection
        '''
        self.mySocket.close()
        print("pluto disconnected")

    def trim(self,Roll,Pitch,Throttle,Yaw):
        '''
        Function to set the trim values to make the drone stable
        '''
        Roll=max(-100,min(Roll,100))
        Pitch=max(-100,min(Pitch,100))
        Throttle=max(-100,min(Throttle,100))
        Yaw=max(-100,min(Yaw,100))
        #Update the values
        self.roll=1500 + Roll
        self.pitch=1500 + Pitch
        self.throttle=1500 + Throttle
        self.yaw=1500 + Yaw
        
        self.rc=[self.roll, self.pitch, self.throttle, self.yaw, self.aux1, self.aux2, self.aux3, self.aux4]
    
    def create_sendMSPpacket(self, msg_type, msg):
        '''
        Function to compose and send message packets to the drone
        '''
        self.buffer=bytearray([])                   # data to be sent
        headerArray=bytearray([36,77,60])           # header array "$","M","<"
        self.buffer.extend(headerArray)
        msg_len=2*len(msg)
        self.buffer.append(msg_len)
        self.buffer.append(msg_type)
        if(msg_len>0):
            for b in msg:
                LSB=b%256
                MSB=math.floor(b/256)
                self.buffer.append(LSB)
                self.buffer.append(MSB)
        CRCValue=0
        for b in self.buffer[3:]:
            CRCValue=CRCValue^b
        self.buffer.append(CRCValue)
    
        if(msg_type==200):
            self.buffer_rc=self.buffer[:]
            self.sendPacket(self.buffer)
        else:
            self.sendPacket(self.buffer_rc)
            self.sendPacket(self.buffer)

    def arm(self):  
        '''
        Function to arm the drone
        ''' 
        self.rc[2]=1000
        self.rc[-1]=1500
        self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
        time.sleep(1)
        print("Arming")
       
    def disarm(self):
        '''
        Function to disarm the drone
        '''
        self.rc[2]=1300
        self.rc[-1]=1200
        self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
        time.sleep(1)
       
    def box_arm(self):
        '''
        Function called before takeoff, user does not directly use it
        '''
        self.rc[2]=1500
        self.rc[-1]=1500
        self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
        time.sleep(0.5)
        print("box_arming")
       
    def clamp_rc(self,x:int):
        #Not called by the user
        return max(1000, min(2000,x))

    def roll_speed(self,value,duration=0):
        '''
        Function to set the roll (x-axis movement) to the drone
        '''
        no_of_loops=10*duration 
        self.rc[0]=self.clamp_rc(self.roll + value)    
       
        while(no_of_loops>0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
            no_of_loops=no_of_loops-1
            time.sleep(0.1)
        if(duration==0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
           
    def pitch_speed(self,value,duration=0):
        '''
        Function to set the pitch (y-axis movement) to the drone
        ''' 
        no_of_loops=10*duration 
        self.rc[1]=self.clamp_rc(self.pitch + value)    
       
        while(no_of_loops>0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
            no_of_loops=no_of_loops-1
            time.sleep(0.1)
        if(duration==0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)

    def throttle_speed(self,value,duration=0): 
        '''
        Function to set the throttle (z-axis movement) to the drone
        '''
        no_of_loops=10*duration 
        self.rc[2]=self.clamp_rc(self.throttle + value)    
       
        while(no_of_loops>0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
            no_of_loops=no_of_loops-1
            time.sleep(0.1)
        if(duration==0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
        
    def yaw_speed(self,value,duration=0):
        '''
        Function to set the yaw (rotation about z-axis) to the drone
        '''
        no_of_loops=10*duration 
        self.rc[3]=self.clamp_rc(self.yaw + value)    
       
        while(no_of_loops>0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
            no_of_loops=no_of_loops-1
            time.sleep(0.1)
        if(duration==0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)

    def set_all_speed(self,roll,pitch,throttle,yaw,duration=0):
        no_of_loops=10*duration
        self.rc[0]=self.clamp_rc(self.roll + roll)
        self.rc[1]=self.clamp_rc(self.pitch + pitch)
        self.rc[2]=self.clamp_rc(self.throttle + throttle)
        self.rc[3]=self.clamp_rc(self.yaw + yaw)
        while(no_of_loops>0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
            no_of_loops=no_of_loops-1
            time.sleep(0.1)
        if(duration==0):
            self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)
           
    def reset_speed(self):
        '''
        Function to reset the roll, pitch, throttle, yaw values for the drone
        '''
        self.rc[:4]=[self.roll,self.pitch,self.throttle,self.yaw]
        self.create_sendMSPpacket(MSP_SET_RAW_RC,self.rc)

    def takeoff(self):
        '''
        Function to takeoff the drone 
        '''
        print("Taking off!!")
        self.box_arm()
        cmd=[1]
        self.create_sendMSPpacket(MSP_SET_COMMAND,cmd)
        self.throttle_speed(0,3)
        print("Taking off!!")
        
    def land(self):
        '''
        Function to land the drone
        '''  
        cmd=[2]
        self.create_sendMSPpacket(MSP_SET_COMMAND,cmd)
        self.throttle_speed(0,5)
        self.disarm()

    def flip(self):
        '''
        Function for backflip
        '''
        print("Get Ready for the FLIP !!")
        cmd=[3]
        self.create_sendMSPpacket(MSP_SET_COMMAND,cmd)
        self.throttle_speed(0,3)

    def read16(self,arr):
        '''
        Function to unpack the byte array to extract the values
        Will not be used by the user directly
        '''
        if((arr[1]&0x80) ==0):
            return ((arr[1] << 8) + (arr[0]&0xff))             # for positive values 
        else:
            return (-65535 + (arr[1] << 8) + (arr[0]&0xff))    # for negative values

    
    ################################################## MSP_ALTITUDE #############################################################


    def get_height(self):
        '''
        Function to return the value of height from the sensors of the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_ALTITUDE,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=109 ):
                i+=1
            if(i+3<len(data)):
                height = self.read16(data[i+1:i+3])
                print(f"height: {height} cm")
                return height


    def get_vario(self):
        '''
        Function to return the value of rate of change of altitude from the sensors of the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_ALTITUDE,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=109 ):
                i+=1
            if(i+7<len(data)):
                return self.read16(data[i+5:i+7])

    
    ###################################################### MSP_ATTITUDE #########################################################

    def get_roll(self):
        '''
        Function to return the value of roll from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_ATTITUDE,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=108 ):
                i+=1
            if(i+3<len(data)):
                roll =  self.read16(data[i+1:i+3])/10
                print(f"pitch: {roll} degrees")
                return roll

    def get_pitch(self):
        '''
        Function to return the value of pitch from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_ATTITUDE,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=108 ):
                i+=1
            if(i+5<len(data)):
                pitch = self.read16(data[i+3:i+5])/10
                print(f"roll: {pitch} degrees")
                return pitch

    def get_yaw(self):
        '''
        Function to return the value of yaw from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_ATTITUDE,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=108 ):
                i+=1
            if(i+7<len(data)):
                return self.read16(data[i+5:i+7])


    ###################################################### MSP_RAW_IMU ##########################################################
    

    def get_acc_x(self):
        '''
        Function to return the value of accelerometer(x-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+3<len(data)):
                return self.read16(data[i+1:i+3])

    def get_acc_y(self):
        '''
        Function to return the value of accelerometer(y-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+5<len(data)):
                return self.read16(data[i+3:i+5])

    def get_acc_z(self):
        '''
        Function to return the value of accelerometer(z-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+7<len(data)):
                return self.read16(data[i+5:i+7])

    def get_gyro_x(self):
        '''
        Function to return the value of gyrometer(x-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+9<len(data)):
                return self.read16(data[i+7:i+9])

    def get_gyro_y(self):
        '''
        Function to return the value of gyrometer(y-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+11<len(data)):
                return self.read16(data[i+9:i+11])

    def get_gyro_z(self):
        '''
        Function to return the value of gyrometer(z-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+13<len(data)):
                return self.read16(data[i+11:i+13])
    
    def get_mag_x(self):
        '''
        Function to return the value of magnetometer(x-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+15<len(data)):
                return self.read16(data[i+13:i+15])
    
    def get_mag_y(self):
        '''
        Function to return the value of magnetometer(y-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+17<len(data)):
                return self.read16(data[i+15:i+17])

    def get_mag_z(self):
        '''
        Function to return the value of magnetometer(z-axis) from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_RAW_IMU,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=102 ):
                i+=1
            if(i+19<len(data)):
                return self.read16(data[i+17:i+19])
###################################################### MSP_calibration ###############################################################

    def calibrate_acceleration(self):
        """
        Function to calibrate the accelerometer.
        """
        # Send the calibration command to the drone
        self.create_sendMSPpacket(MSP_ACC_CALIBRATION, [])

        # Wait for calibration to finish
        time.sleep(5)  # Adjust sleep duration based on calibration time

        # Optionally, you can read and print the accelerometer values to verify calibration
        acc_x = self.get_acc_x()
        acc_y = self.get_acc_y()
        acc_z = self.get_acc_z()
        print("Calibrated Accelerometer Values:")
        print(f"X-Axis: {acc_x}, Y-Axis: {acc_y}, Z-Axis: {acc_z}")

    def calibrate_magnetometer(self):
        """
        Function to calibrate the magnetometer.
        """
        # Send the calibration command to the drone
        self.create_sendMSPpacket(MSP_MAG_CALIBRATION, [])

        # Wait for calibration to finish
        time.sleep(5)  # Adjust sleep duration based on calibration time

        # Optionally, you can read and print the magnetometer values to verify calibration
        mag_x = self.get_mag_x()
        mag_y = self.get_mag_y()
        mag_z = self.get_mag_z()
        print("Calibrated Magnetometer Values:")
        print(f"X-Axis: {mag_x}, Y-Axis: {mag_y}, Z-Axis: {mag_z}")


###################################################### MSP_ANALOG ###############################################################


    def get_battery(self):
        '''
        Function to return the value of battery in volts from the drone
        '''
        data=[]
        self.create_sendMSPpacket(MSP_ANALOG,data) 
        for i in range(RETRY_COUNT):
            data=self.recievePacket()
            i=0
            while(i<len(data) and data[i]!=110 ):
                i+=1
            if(i+1<len(data)):
                battery_value = data[i + 1] / 10
                print(f"Battery: {battery_value} volts")
                return battery_value

   

    '''Function to send and recieve data packets'''
    def sendPacket(self,buff):
        self.mySocket.send(buff)

    def recievePacket(self):
       return self.mySocket.recv(self.BUFFER_SIZE)