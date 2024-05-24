import numpy as np
import math
import time



class MoteurCC:
    def __init__(self, res = 1, inductance = 0.001, kc = 0.01, ke = 0.01, rotorJ = 0.01, resistive_torque=0, externJ=0, max_voltage=0, f=0.1, start_pos=0):

        ## MOTOR PARAMS
        self.resistance_Ohm = res
        self.inductance_H = inductance
        self.backemf = 0

        self.max_voltage_V = max_voltage

        self.torque_constant_Nm_A = kc
        self.backemf_constant_Vs = ke

        self.rotor_inertia_kgm2 = rotorJ
        self.total_inertia_kgm2 = rotorJ + externJ

        self.viscous_friction_Nms = f

        self.resistive_torque_Nm = resistive_torque

        self.voltage_V = [0]
        self.current_A = [0] 
        self.torque_Nm = [0]
        self.speed_rad_s = [0]
        self.position_rad = [start_pos] 

        ## CONSTANTS FOR ANALYTICAL SOLUTION
        denom = 1 / (self.torque_constant_Nm_A * self.backemf_constant_Vs + self.resistance_Ohm * self.viscous_friction_Nms)
        self.tau = (self.resistance_Ohm * self.total_inertia_kgm2) * denom
        self.K = self.torque_constant_Nm_A * denom

    def __str__(self):
        return "Moteur CC (Speed : "+str(self.speed_rad_s[-1])+', Torque : '+str(self.torque_Nm[-1])+', Voltage '+self.voltage_V[-1]+', Current '+str(self.current_A[-1])+')'

    def __repr__(self):
        return str(self)
    
    def setExternTorque(self, extern_torque):
        self.resistive_torque_Nm = extern_torque
    
    def setInertia(self, load_inertia):
        
        """
            set inertia (J)
        """
        self.total_inertia_kgm2 = self.rotor_inertia_kgm2 + load_inertia

        denom = 1 / (self.torque_constant_Nm_A * self.backemf_constant_Vs + self.resistance_Ohm * self.viscous_friction_Nms)
        self.tau = (self.resistance_Ohm * self.total_inertia_kgm2) * denom
        self.K = self.torque_constant_Nm_A * denom

    def setPosition(self, position):
        self.position_rad[-1] = position
        

    def setViscosity(self, visco):
        self.viscous_friction_Nms = visco

        denom = 1 / (self.torque_constant_Nm_A * self.backemf_constant_Vs + self.resistance_Ohm * self.viscous_friction_Nms)
        self.tau = (self.resistance_Ohm * self.total_inertia_kgm2) * denom
        self.K = self.torque_constant_Nm_A * denom
    
    def setVoltage(self, voltage):
        if self.max_voltage_V:
            voltage = min(self.max_voltage_V, voltage)
        self.voltage_V.append(voltage)

    def getCurrent(self):
        return self.current_A[-1]
    
    def getSpeed(self):
        return self.speed_rad_s[-1]

    def getTorque(self):
        return self.torque_Nm[-1]
    
    def getPosition(self):
        return self.position_rad[-1]
    
    def getAllStates(self):
        return self.position_rad, self.speed_rad_s, self.torque_Nm, self.current_A
    
    
    def d_dt(self,current,speed):
        
        """
            Derivatives for current and speed (see formula given by S. Haliyo)
        """

        didt = (self.voltage_V[-1] - self.resistance_Ohm * current - self.backemf_constant_Vs * speed) / self.inductance_H
        dodt = (self.torque_constant_Nm_A * current - self.viscous_friction_Nms * speed  - self.resistive_torque_Nm) / self.total_inertia_kgm2
        
        # print(current,speed,dodt,didt)
        return [dodt, didt]

    def rk4(self, current, speed):

        """
            Parallel RK4 to numerically integrate the DC motor equation.
        """

        #the o suffix is for omega, as in the speed
        #the i suffix is for the current
        derivatives = self.d_dt(current, speed)
        k1_o = derivatives[0] * self.stepsize_s
        k1_i = derivatives[1] * self.stepsize_s

        derivatives = self.d_dt(current + 0.5*k1_i, speed + 0.5*k1_o)
        k2_o = self.stepsize_s * derivatives[0]
        k2_i = self.stepsize_s * derivatives[1]

        derivatives = self.d_dt(current + 0.5*k2_i, speed + 0.5*k2_o)
        k3_o = self.stepsize_s * derivatives[0]
        k3_i = self.stepsize_s * derivatives[1]

        derivatives = self.d_dt(current+ k3_i, speed+k3_o)
        k4_o = self.stepsize_s * derivatives[0]
        k4_i = self.stepsize_s * derivatives[1]

        new_speed = speed + 1/6. * (k1_o + 2*k2_o + 2*k3_o + k4_o)
        new_current = current + 1/6. * (k1_i + 2*k2_i + 2*k3_i + k4_i)
        # time.sleep(3)
        return new_current, new_speed
    

    def simule(self, step):

        """
            Simulate a step. Numerically integrate current and speed, then integrate speed to get position. 
        """

        self.stepsize_s = step
        new_curr, new_speed = self.rk4(self.current_A[-1], self.speed_rad_s[-1])
        # print(new_curr, new_speed)
        # time.sleep(1)
        
        self.current_A.append(new_curr)
        self.speed_rad_s.append(new_speed)
        self.torque_Nm.append(self.torque_constant_Nm_A * new_curr - self.resistive_torque_Nm)
        self.position_rad.append(self.position_rad[-1] + new_speed*self.stepsize_s)

    def analytical_solution(self, time):
        return (self.K * (1 - np.exp(-time / self.tau))*self.voltage_V[-1])


class PIDController:
    def __init__(self, P,I,D, motor:MoteurCC, max_windup=0, noise_stddev=0):

        self.motor = motor
        self.Kp, self.Ki, self.Kd = P,I,D
        self.max_windup = max_windup

        if I and not max_windup:
            print("[WARNING - PIDController] Max Windup not set")

        self.old_error = 0
        
        self.integral = 0

        self.noise = noise_stddev #Add 0 mean gaussian noise to the errors to simulate a more realistic environment

    def simule(self, step, target, measurement):

        error = target - measurement

        if self.noise:
            error += np.random.normal(scale=self.noise)

        self.integral += error * step

        if self.max_windup:
            self.integral = min(max(-self.max_windup, self.integral), self.max_windup)

        if self.Kd:
            derivative = (error - self.old_error)/step
            self.old_error = error
        else:
            derivative = 0

        output_V = self.Kp * error + self.Kd * derivative + self.Ki * self.integral
        self.motor.setVoltage(output_V)
        self.motor.simule(step)



def test_proper_behavior():
    
    """
        Some unit tests (not using any libraries, just some manual ones)
    """
    #Test proper movement
    motor = MoteurCC()
    initial_speed = motor.getSpeed()
    initial_pos = motor.getPosition()

    motor.setVoltage(10)
    motor.simule(0.1)

    test1 = (initial_speed != motor.getSpeed())
    test2 = (initial_pos != motor.getPosition())

    #Test RK4
    motor = MoteurCC()
    initial_current = 0
    initial_speed = 0 
    motor.setVoltage(10)
    motor.stepsize_s = 0.1
    new_curr, new_speed = motor.rk4(initial_current, initial_speed)

    test3 = bool(new_curr)
    test4 = bool(new_speed)

    return all([test1,test2,test3,test4])


def test_simulation():

    """
        As is written in the sujet
    """

    TestMotor = MoteurCC(inductance=0.1)
    
    t = 0
    step = 0.001
    temps = [t]
    speeds = [0]

    while t<1: 
        t+= step
        temps.append(t)
        TestMotor.setVoltage(100)
        speeds.append(TestMotor.analytical_solution(temps[-1]))
        TestMotor.simule(step)
        
        
    
    plt.figure("Step Response")
    plt.title("Step response comparaison of numerical and analytical solutions")
    plt.plot(temps, TestMotor.speed_rad_s, label=f"RK4 solution (L={TestMotor.inductance_H}H)")
    plt.plot(temps, speeds, label="Analytical solution (L=0H)")
    plt.xlabel("Temps (s)")
    plt.legend()
    plt.show()

def test_speedcontrol(target_speed):
    TestMotor = MoteurCC(max_voltage=0)
    pid_speedcontrol = PIDController(P=80.0, I=3000, D=0.5, max_windup=50, motor=TestMotor, noise_stddev = 0.02)

    t = 0
    step = 0.001
    temps = [t]

    rise_time = None
    

    while t<0.15: 
        t+= step
        temps.append(t)
        pid_speedcontrol.simule(step, target_speed, TestMotor.getSpeed())   

        # if t>0.5:
        #     TestMotor.setExternTorque(40)

        if rise_time is None and (TestMotor.getSpeed() > 0.95 * target_speed):
            rise_time = temps[-1]

    print("max speed speed control: ", max(TestMotor.speed_rad_s), " rad/s")

    if rise_time is None:
        rise_time = float('inf')
    plt.figure("PID Speed Control")
    plt.ylim(-10,30)
    plt.title(f"PID Speed Control, target={target_speed}, static error={(TestMotor.getSpeed()-target_speed):.2f}, rise time = {rise_time:.2f}s")
    plt.plot(temps, [i * 0.1 for i in TestMotor.voltage_V], label="Voltage (0.1V)", color="green")
    plt.plot(temps, TestMotor.speed_rad_s, label="Speed (rad/s)")
    plt.plot(temps, TestMotor.torque_Nm, label="Torque (Nm)")
    plt.legend()
    plt.show()

def test_poscontrol(target_pos):

    TestMotor = MoteurCC(max_voltage=5000)
    pid_speedcontrol = PIDController(P=80.0, I=1, D=5, max_windup=2, motor=TestMotor, noise_stddev=0.02)

    t = 0
    step = 0.001
    temps = [t]
    rise_time = None

    while t<1.5: 
        t+= step
        temps.append(t)
        pid_speedcontrol.simule(step, target_pos, TestMotor.getPosition())
        if rise_time is None and (TestMotor.getPosition() > 0.95 * target_pos):
            rise_time = temps[-1]
        # if t>2:
        #     TestMotor.setExternTorque(5)

    if rise_time is None:
        rise_time = float('inf')

    print("max speed pos control: ", max(TestMotor.speed_rad_s), " rad/s")
    
    plt.figure("PID Pos Control")
    plt.title(f"PID Pos Control, target={target_pos}, final error={(TestMotor.getPosition()-target_pos):.2f}, rise time = {rise_time:.2f}s")
    plt.ylim(-50,50)
    plt.plot(temps, [i * 0.1 for i in TestMotor.voltage_V], label="Voltage (0.1V)", color="green")
    plt.plot(temps, TestMotor.position_rad, label="Position (rad)")
    plt.plot(temps, TestMotor.torque_Nm, label="Torque (Nm)")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    if not test_proper_behavior():
        print("The Motor class failed its unit tests")

    test_poscontrol(10)
    test_simulation()
    test_speedcontrol(4)






    