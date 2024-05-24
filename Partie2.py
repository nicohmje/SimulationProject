from particule import * 
from Partie1 import * 
from vecteur3D import *

import numpy as np 

class AngleTracker:
    def __init__(self):
        self.old_angle = 0
        self.total_rotation = 0

    def update_angle(self, axis_x, axis_y):
        # Calculate new angle
        new_angle = -1 * math.atan2(axis_x, axis_y)
        
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

"""
    The objective here is to simulate a particle attached by means of a linear spring to a motor.
    The faster the motor spins, the further away the particule will be.
    
    We want to plot d as a function of Omega, the rotational speed of the motor.


    We will make two particules, one fixed which will be the center point of the motor, and one moving. 

    The two will be attached by a Prismatic link.

"""

angle = 0

def customEvents(univers:Univers):
    pass
    univers.controller.simule(univers.step, sim_env.target, univers.measured_speeds[-1])

    position = univers.motor.getPosition()
    speed = univers.motor.getSpeed()
    torque = univers.motor.getTorque()
    axis = (P_motor.getPosition() - P_pend.getPosition())
    tangent = Vecteur3D(-axis.y, axis.x, 0).norm()

    distance = axis.mod() * 0.01

    global old_angle
    global angle
    old_angle = angle
    angle = sim_env.angletracker.update_angle(axis.x, axis.y)

    P_pend.applyForce(-torque/distance * tangent)

    print("\n torque: ", torque)
    print("position: ", position)
    print("estimated speed: ", speed)

    


    
    measured_speed = (angle - old_angle) / univers.step

    print("distance: ", distance)
    print("measured angle: ", angle)
    print("measured speed: ", measured_speed)

    univers.motor.setInertia(P_pend.mass * distance**2 * 100)

    new_axis = Vecteur3D(math.cos(angle), math.sin(angle), 0)
    f_link.axis = new_axis.norm()



    gravity_torque = math.sin(angle) * P_pend.mass * 10 * distance * 10
    print(f"gravity toque {gravity_torque}")
    univers.motor.setExternTorque(gravity_torque)
    
    
    
    
    

    univers.estimated_speeds.append(speed)
    univers.measured_speeds.append(measured_speed)
    univers.dists.append(distance)
    pass


if __name__ == "__main__":
    sim_env = Univers(name='Part 2',step=0.0001)

    P_motor = Particule(p0=Vecteur3D(500,250,0),fix=True,name="DC Motor",color="white")
    P_pend = Particule(p0=Vecteur3D(500,150,0),name="pendulum",color="cyan", mass=1)


    motor = MoteurCC(max_voltage=50000, f=4)

    # pidcontrol = PIDController(P=1000.0, I=0, D=5000, max_windup=2, motor=motor) # Pos gains
    pidcontrol = PIDController(P=1000.0, I=100, D=0, max_windup=1000, motor=motor) # Pos gains

    sim_env.addAgent(P_motor,P_pend)

    sim_env.angletracker = AngleTracker()

    f_down = Gravity(Vecteur3D(0,-10,0),active=True)

    # k = 400
    # c= 1000

    k = 100
    c = 100

    axis = (P_motor.getPosition() - P_pend.getPosition()).rotZ(math.pi/2)
    f_link = Prismatic(P_motor,P_pend, axis.norm(), k=k, c=c)

    sim_env.addGenerators(f_link, f_down)

    sim_env.motor = motor
    sim_env.controller = pidcontrol

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

    sim_env.plot()

    from pylab import figure,plot,show,legend,xlabel,ylabel,title

    figure()
    title(f"Estimated vs Measured (Target speed=8 rad/s, k={k}, c={c})")
    plot(sim_env.measured_speeds,sim_env.estimated_speeds)
    plot(sim_env.measured_speeds,sim_env.measured_speeds,label="y=x")
    xlabel("measured (rad/s)")
    ylabel("estimated (m)")
    legend()
    show()


    figure()
    title(f"Spring elongation as a function of the rotor's speed (k={k})")
    plot(sim_env.measured_speeds,sim_env.dists,label="Measured speed")
    plot(sim_env.estimated_speeds,sim_env.dists,label="Estimated speed")
    xlabel("speed (rad/s)")
    ylabel("elongation (m)")
    legend()
    show()

    