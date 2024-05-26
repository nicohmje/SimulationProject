from particule import * 
from Partie1 import * 
from vecteur3D import *

import numpy as np 

"""
    The objective here is to simulate a particle attached by means of a linear spring to a motor.
    The faster the motor spins, the further away the particule will be.
    
    We want to plot d as a function of Omega, the rotational speed of the motor.

    We will make two particules, one fixed which will be the center point of the motor, and one moving. 

    The two will be attached by a Prismatic link.
"""

class AngleTracker:

    """
        Just a ChatGPT created class to keep track of an angle even as it goes over 2pi. 
    """

    def __init__(self):
        self.old_angle = 0
        self.total_rotation = 0

    def update_angle(self, axis_1, axis_2):
        # Calculate new angle
        new_angle = -1 * math.atan2(axis_1, axis_2)
        
        # Calculate change in angle
        angle_delta = new_angle - self.old_angle

        # Correct for the discontinuity at -pi and pi
        if angle_delta > math.pi:
            angle_delta -= 2 * math.pi
        elif angle_delta < -math.pi:
            angle_delta += 2 * math.pi

        # Update total rotation
        self.total_rotation += angle_delta

        # Update old angle for the next update
        self.old_angle = new_angle

        # The total_rotation attribute can grow beyond [-pi, pi]
        return self.total_rotation

angle = 0 #Global variable

def customEvents(univers:Univers):

    """
        We measure the angle from the particle to the motor, differentiate it to get speed,
        then pass it to the controllers for speed or position control. 

        We also update the prismatic spingdamper axis, the inertia of the particle for the Motor,
        and finally apply gravity torque to the motor. We also log the elongation of the spring. 
    """

    global angle


    speed = univers.motor.getSpeed()
    torque = univers.motor.getTorque()
    axis = (P_motor.getPosition() - P_pend.getPosition()) 
    tangent = Vecteur3D(-axis.y, axis.x, 0).norm() 
    distance = axis.mod() * 0.01 #convert to cm

    old_angle = angle
    angle = sim_env.angletracker.update_angle(axis.x, axis.y)
    measured_speed = (angle - old_angle) / univers.step # Differntiate for speed

    sim_env.measured_angles.append(angle)

    P_pend.applyForce(-torque/distance * tangent)


    # print("\nmeasured angle: ", angle)
    # print("measured speed: ", measured_speed)

    univers.motor.setInertia(P_pend.mass * distance**2 * 100) # Mr^2

    new_axis = Vecteur3D(math.cos(angle), math.sin(angle), 0)
    f_link.axis = new_axis.norm()


    # Compute torque from gravity

    gravity_torque = math.sin(angle) * P_pend.mass * 10 * distance * 10 
    univers.motor.setExternTorque(gravity_torque)
    
    # print(f"gravity toque {gravity_torque}")

    if sim_env.choice == "1":
        univers.dists.append(distance)
        univers.measured_speeds.append(measured_speed)
        univers.controller.simule(univers.step, sim_env.target, univers.measured_speeds[-1])


        if univers.time[-1]% (univers.step*500) <1e-3:
            print(f"\nMeasured speed: {measured_speed:.3f}rad/s")

    elif sim_env.choice == "2":
        univers.controller.simule(univers.step, sim_env.target, angle)
        if univers.time[-1]% (univers.step*500) <1e-3:
            print(f"\nMeasured angle: {angle:.3f}rad")
    else:
        univers.estimated_speeds.append(speed)
        univers.measured_speeds.append(measured_speed)
        univers.motor.setVoltage(10000)
        univers.motor.simule(univers.step)
        if univers.time[-1]% (univers.step*500) <1e-3:
            print(f"\nElongation: {distance:.3f}m")
    pass


if __name__ == "__main__":
    sim_env = Univers(name='Part 2',step=0.0001)

    P_motor = Particule(p0=Vecteur3D(500,250,0),fix=True,name="DC Motor",color="white")
    P_pend = Particule(p0=Vecteur3D(500,150,0),name="pendulum",color="cyan", mass=1)
    

    while True:

        print("\n\n\n\n\n\n\nChoisissez la demo a effectuer:    ")
        print(" 1. Controle en vitesse (8 rad/s) + Elongation du ressort")
        print(" 2. Controle en position (8 rad) ")
        print(" 3. Validation du modele (vitesse du moteur vs vitesse de la particule)")
        
        choice = input("Choice ? : \n ")

        if choice == "1":
            break
        elif choice == "2":
            break
        elif choice == "3":
            break
        else:
            print("\n\n Choix incorrect.")
    sim_env.choice = choice
    motor = MoteurCC(max_voltage=50000, f=4)
    sim_env.addAgent(P_motor,P_pend)

    sim_env.angletracker = AngleTracker()

    if choice == "1":
        pidcontrol = PIDController(P=4000.0, I=100, D=300, max_windup=20, motor=motor) # Speed gains
        sim_env.controller = pidcontrol
    elif choice == "2":
        pidcontrol = PIDController(P=8000.0, I=1200, D=20000, max_windup=10, motor=motor) # Pos gains
        sim_env.controller = pidcontrol



    f_down = Gravity(Vecteur3D(0,-10,0),active=True)

    k = 500 #400
    c= 0 #1000

    # k = 100
    # c = 100

    axis = (P_motor.getPosition() - P_pend.getPosition()).rotZ(math.pi/2)
    f_link = Prismatic(P_motor,P_pend, axis.norm(), k=k, c=c)

    sim_env.addGenerators(f_link, f_down)

    sim_env.motor = motor

    sim_env.measured_angles = [0]
    sim_env.measured_speeds = [0]
    sim_env.estimated_speeds = [0]
    sim_env.dists =[1]

    sim_env.target = 8

    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))


    ###############################
    #
    # Post-traitement
    #


    from pylab import figure,plot,show,legend,xlabel,ylabel,title, xlim, ylim
    
    if choice == "1":
        figure("PID Speed control")
        title(f"PID Speed Control (target = {sim_env.target}, final error = {sim_env.measured_speeds[-1] - sim_env.target:.3f})")
        plot(sim_env.time,sim_env.measured_speeds,label="Measured speed")
        xlabel("time (s)")
        ylabel("angular velocity (rad/s)")
        legend()
        show()

        figure("Spring elongation")
        title(f"Spring elongation as a function of the rotor's speed (k={k})")
        plot(sim_env.measured_speeds,sim_env.dists,label="Measured speed")
        xlabel("speed (rad/s)")
        ylabel("elongation (m)")
        legend()
        show()

    elif choice == "2":
        figure("PID Pos control")
        title(f"PID Pos Control (target = {sim_env.target}, final error = {sim_env.measured_angles[-1] - sim_env.target:.3f})")
        plot(sim_env.time,sim_env.measured_angles,label="Measured angle")
        xlabel("time (s)")
        ylabel("angle (rad)")
        legend()
        show()
    elif choice == "3":
        figure("Estimated vs Measured speeds")
        title(f"Estimated vs Measured (k={k})")
        plot(sim_env.time,sim_env.estimated_speeds, label="estimated")
        plot(sim_env.time,sim_env.measured_speeds, label="measured")
        xlabel("Time (s/s)")
        ylabel("Speed (rad /s)")
        legend()
        show()

        figure("Estimated vs Measured speeds")
        title(f"Estimated vs Measured (k={k}, c={c})")
        plot(sim_env.measured_speeds,sim_env.estimated_speeds)
        plot(sim_env.measured_speeds,sim_env.measured_speeds,label="y=x")
        xlabel("Measured (rad/s)")
        ylim(0,10)
        xlim(0,10)
        ylabel("estimated (rad /s)")
        legend()
        show()
