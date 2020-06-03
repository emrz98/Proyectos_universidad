import time
import RPi.GPIO as GPIO
import wiringpi as wp
import rospy,time
from i2cpwm_board.msg import Servo,ServoArray
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialArray
#############################################################################      Clase ultrasonico
class ultrasonico():
    def __init__(self,trigger,echo, dmax=400,dmin=100):
        
        GPIO.setmode(GPIO.BCM)
        self.trigger=trigger
        self.echo=echo
        self.dmax=dmax
        self.dmin=dmin
        self.velSonido = 17150.0
        self._timeout = dmax/self.velSonido*2
        GPIO.setup(trigger,GPIO.OUT)
        GPIO.setup(echo,GPIO.IN)
        
        GPIO.output(trigger,GPIO.LOW)
        self._is_reading = False
        
        
    def distancia(self):
        self._is_reading = True
        GPIO.output(self.trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger, GPIO.LOW)
        
        GPIO.output(self.trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger, GPIO.LOW)

        
        pulse_start_time = time.time()
        pulse_end_time = time.time()
        #--- Wait for the answer
        while GPIO.input(self.echo)==0:
            pulse_start_time = time.time()
           # if time.time() - time0 > self._timeout:
             
            #    self._is_reading = False
             #   print time.time() - time0
              #  print("TIMEOUT")
               # return (-1)
            #time0= time.time()
        while GPIO.input(self.echo)==1:
            pulse_end_time = time.time()
            
            
        self._last_time_reading = time.time()
        self._is_reading = False

        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * self.velSonido
        return distance
    
###########################################################################################        Clase PWM
class ServoConvert:
    def __init__(self,id):
        self.id=id
        self.value_out=0
#########################################################################################          Clase Seguidor de linea

class MrBit_QTR_8RC:
  
 

    def __init__(self):
       
        #Inicializar motores y servo
        self.actuators = {}
        self.actuators['MotorDerecho']  = ServoConvert(id=14)
        self.actuators['MotorIzquierdo']  = ServoConvert(id=13) 
        self.actuators['ServoBasura']= ServoConvert(id=16)
        self._servo_msg       = ServoArray()
        for i in range(16): self._servo_msg.servos.append(Servo())
        self._ros_sub_aruco          = rospy.Subscriber("/fiducial_vertices", FiducialArray, self.callback)
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        self.idAruco=0
   

        #Inicializar seguidor de linea
        
        GPIO.setmode(GPIO.BCM)
        self.wp = wp
        self.lastError=0
        self.errorI=0
        self.lastProporcional=0
        self.integral=0
        self.KP=2
        #self.KI=0.001
        self.KD=5

        self.velMinIzquierda=1500
        self.velMinDerecha=1500
        self.velMaxIzquierda=2500
        self.velMaxDerecha=2500
        
        self.Foco=20
        self.Boton=6
        GPIO.setup(self.Foco, GPIO.OUT)
        GPIO.output(self.Foco, GPIO.HIGH)
        

         
        self.LEDON_PIN = 5
        self.SENSOR_PINS = [26, 23, 24, 25, 12]#6
        self.NUM_SENSORS = len(self.SENSOR_PINS)
        self.CHARGE_TIME = 10 #us to charge the capacitors
        self.READING_TIMEOUT = 1000 #us, assume reading is black
 
        self.sensorValues = []
        self.calibratedMax = []
        self.calibratedMin = []
        self.lastValue = 0
        self.init_pins()
 
    def callback(self,datos):
	self.idAruco=datos.fiducials[0].fiducial_id
        print(self.idAruco)

    def getAruco(self):
        return self.idAruco

    def init_pins(self):
       
        for pin in self.SENSOR_PINS:
            self.sensorValues.append(0)
            self.calibratedMax.append(0)
            self.calibratedMin.append(0)
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        GPIO.setup(self.LEDON_PIN,GPIO.OUT)
 
    def emitters_on(self):
      
        GPIO.output(self.LEDON_PIN,GPIO.HIGH)
        self.wp.delayMicroseconds(20)
 
 
    def emitters_off(self):
       
        
        GPIO.output(self.LEDON_PIN,GPIO.LOW)
        for sensorPin in self.SENSOR_PINS:
            GPIO.setup(sensorPin, GPIO.OUT)
        self.wp.delayMicroseconds(20)
  
 
    def print_sensor_values(self, values):
       
        for i in range(0, self.NUM_SENSORS):
            print("sensor %d, reading %d" % (i, values[i]))
 
        for sensorPin in self.SENSOR_PINS:
            GPIO.setup(sensorPin,GPIO.OUT)
    def initialise_calibration(self):
       
        for i in range(0, self.NUM_SENSORS):
            self.calibratedMax[i] = 0
            self.calibratedMin[i] = self.READING_TIMEOUT
 
 
    def calibrate_sensors(self):
        
        for j in range(0, 10):
            self.read_sensors()
            for i in range(0, self.NUM_SENSORS):
                if self.calibratedMax[i] < self.sensorValues[i]:
                    self.calibratedMax[i] = self.sensorValues[i]
                if self.calibratedMin[i] > self.sensorValues[i] and self.sensorValues[i] > 30:
                    self.calibratedMin[i] = self.sensorValues[i]
 
 
    def read_line(self):
      
        self.read_calibrated()
 
        avg = 0
        summ = 0
        online = False
 
        for i in range(0, self.NUM_SENSORS):
            val = self.sensorValues[i]
            if val > 400: online = True#500
            if val > 50:#50
                multiplier = i * 1000
                avg += val * multiplier
                summ +=  val
 
        if online == False:
            if self.lastValue < (self.NUM_SENSORS-1)*1000/2:
                return 0
            else:
                return (self.NUM_SENSORS-1)*1000
 
        self.lastValue = avg/summ
        return self.lastValue
 
 
    def read_calibrated(self):
        """ Reads the calibrated values for each sensor.
        """
 
        self.read_sensors()
 
        print("uncalibrated readings")
        self.print_sensor_values(self.sensorValues)
 
        for i in range(0, self.NUM_SENSORS):
            denominator = self.calibratedMax[i] - self.calibratedMin[i]
            val = 0
            if denominator != 0:
                val = (self.sensorValues[i] - self.calibratedMin[i]) * 1000 / denominator
            if val < 0:
                val = 0
            elif val > 1000:
                val = 1000
            self.sensorValues[i] = val
 
        print("calibrated readings")
        self.print_sensor_values(self.sensorValues)
 
 
    def read_sensors(self):
        """ Follows the Pololu guidance for reading capacitor discharge/sensors:
            1. Set the I/O line to an output and drive it high.
            2. Allow at least 10 us for the sensor output to rise.
            3. Make the I/O line an input (high impedance).
            4. Measure the time for the voltage to decay by waiting for the I/O
                line to go low.
            Stores values in sensor values list, higher vals = darker surfaces.
        """ 
        
        for i in range(0, self.NUM_SENSORS):
            self.sensorValues[i] = self.READING_TIMEOUT
 
        for sensorPin in self.SENSOR_PINS:
            GPIO.setup(sensorPin,GPIO.OUT)
            GPIO.output(sensorPin,GPIO.HIGH)
 
        self.wp.delayMicroseconds(self.CHARGE_TIME)
        for sensorPin in self.SENSOR_PINS:
            GPIO.setup(sensorPin, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
            #important: ensure pins are pulled down
            
 
        startTime = self.wp.micros()
        while self.wp.micros() - startTime < self.READING_TIMEOUT:
            time = self.wp.micros() - startTime
            for i in range(0, self.NUM_SENSORS):
                   
                if  GPIO.input(self.SENSOR_PINS[i]) == 0 and time < self.sensorValues[i]:
                    self.sensorValues[i] = time
 
    def correc_error(self):
	 
        self.val=self.read_line()
        self.error=self.val-2500
        #self.errorI=self.errorI+self.error
        self.VelMotor=self.KP*self.error+self.KD*(self.error-self.lastError)
        #KD*(self.error-self.lastError)
        #+self.KI*(self.errorI)
 	self.lastError=self.error
      
        self.actuators['MotorIzquierdo'].value_out=self.velMinIzquierda+self.VelMotor
        self.actuators['MotorDerecho'].value_out=self.velMinDerecha-self.VelMotor
      
        if(self.actuators['MotorIzquierdo'].value_out>self.velMaxIzquierda): 
            self.actuators['MotorIzquierdo'].value_out=self.velMaxDerecha

        if( self.actuators['MotorDerecho'].value_out>self.velMaxDerecha):
            self.actuators['MotorDerecho'].value_out=self.velMaxDerecha

        if(self.actuators['MotorIzquierdo'].value_out<0):
            self.actuators['MotorIzquierdo'].value_out=0

        if( self.actuators['MotorDerecho'].value_out<0):
            self.actuators['MotorDerecho'].value_out=0
        self.actuators['ServoBasura'].value_out=300
        self.enviarMensaje()

    def frenar(self):
        self.actuators['MotorIzquierdo'].value_out=0
        self.actuators['MotorDerecho'].value_out=0
        self.actuators['ServoBasura'].value_out=425
        self.enviarMensaje()
        wp.delay(5000)

    def enviarMensaje(self):
        for actuator_name, servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(actuator_name, servo_obj.value_out))
        print self.val
        #print self.idAruco

        self.ros_pub_servo_array.publish(self._servo_msg)

    def alertaRojaActivada(self):
        self.actuators['MotorIzquierdo'].value_out=0
        self.actuators['MotorDerecho'].value_out=0
        self.actuators['ServoBasura'].value_out=300
        GPIO.output(self.Foco,GPIO.LOW)

        self.enviarMensaje()


    def levantarTapa():
        self.actuators['MotorIzquierdo'].value_out=0
        self.actuators['MotorDerecho'].value_out=0
        self.actuators['ServoBasura'].value_out=425

    def alertaRojaDesactivada(self):
        GPIO.output(self.Foco,GPIO.HIGH)

    

  #############################################################################################################    PROGRAMA PRINCIPAL       
 
#Example ussage:
if __name__ == "__main__":
    
    rospy.init_node('seguidor')
    qtr = MrBit_QTR_8RC()
    sonar= ultrasonico(4,17)
    sonar2=ultrasonico(27,22)
    GPIO.setup(6, GPIO.IN)
#############################################################################################################     CALIBRACION SEGUIDOR
    try:
        
 
        approveCal = False
        while not approveCal:
            print("calibrating")
            qtr.initialise_calibration()
            qtr.emitters_on()
            for i in range(0, 250):
                qtr.calibrate_sensors()
                wp.delay(20)
            qtr.emitters_off()
 
            print "calibration complete"
            print "max vals"
            qtr.print_sensor_values(qtr.calibratedMax)
            print "calibration complete"
            print "min vals"
            qtr.print_sensor_values(qtr.calibratedMin)
            approved = raw_input("happy with calibrtion (Y/n)? ")
            if approved == "Y": approveCal = True
    except Exception as e:
        qtr.emitters_off()
        print str(e)
 ############################################################################################        BUCLE PROGRAMA PRINCIPAL
    try:
        while not rospy.is_shutdown():
            d=sonar.distancia()
            d2=sonar2.ultrasonico()
            if(d2>30 && GPIO.input(6) == GPIO.HIGH): #y switch apagado
                qtr.alertaRojaDesactivada()
                if(d>20):
                    qtr.emitters_on()
                    qtr.correc_error()
                    qtr.emitters_off()
                if(d<20): 
                    qtr.frenar()
                

            if(d2<30):
 		qtr.alertaRojaActivada()
                if (GPIO.input(6) == GPIO.LOW):#d2<30 y switch encendido   Parar y abrir
                    qtr.levantarTapa()

            if (GPIO.input(6) == GPIO.LOW)#Switch encedido y d2>30         Parar y abrir al mismo tiempo
                qtr.alertaRojaActivada()
                qtr.levantarTapa()

            rospy.sleep(0.01)
    except KeyboardInterrupt:
        qtr.emitters_off()
 
    except Exception as e:
        print str(e)
