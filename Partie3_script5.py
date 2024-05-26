from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
   Pendule inverse

    controle manuel et automatique
"""

def pendulum_coordinates(L, theta):
    """
        Distance + angle -> coordonees
    """

    origin = P_rotor.getPosition()

    theta_radians = math.radians(theta)
    
    x = origin.x + L * math.sin(theta_radians)
    y = origin.y + L * math.cos(theta_radians)
    
    return Vecteur3D(int(x), int(y),0)


class PIDController:

    def __init__(self, Kp, Kd, timestep, Ki=0, windup_lim = 10):
        self.old_error = 0 
        self.error = 0
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.windup_lim = windup_lim
        self.timestep = timestep

        self.integral = 0
        pass

    def control(self, target, measurement):
        self.old_error = self.error
        self.error = target - measurement

        self.integral += self.error

        self.integral = min(max(self.integral, -self.windup_lim), self.windup_lim)

        derivative = (self.error - self.old_error) / self.timestep

        return self.Kp * self.error + self.Kd * derivative + self.Ki * self.integral



angle = 0

def customEvents(univers:Univers):
    axis = (P_rotor.getPosition() - P_pend.getPosition())
    global old_angle
    global angle
    old_angle = angle
    angle = -math.atan2(axis.y, axis.x)
    univers.angle.append(angle)
    tangent = Vecteur3D(-axis.y, axis.x, 0).norm() 
    ang_vel = (old_angle-angle)/univers.step
    cart_velocity = P_rotor.getSpeed().x
    univers.pos1.append(P_rotor.getPosition().x)

    position_error = P_rotor.getPosition().x - 525

    if abs(angle - math.pi/2.) < 0.1 and univers.rise_time1 is None: #within 5 deg 
        univers.rise_time1 = univers.time[-1]

    if abs(angle - math.pi/2.) > 0.04:
        univers.settling_time1 = univers.time[-1]

    if abs(position_error) < 50 and univers.rise_time2 is None: # within 5 cm 
        univers.rise_time2 = univers.time[-1]

    if abs(position_error) > 25: # within 2.5 cm
        univers.settling_time2 = univers.time[-1]




    #Pendulum - Rotor pivot damping torque: 

    # visous friction torque : Tau_f = L_p * ang_vel * c_p

    # Resulting force : F = (Tau_F / L_p) colinear to tangent 

    P_pend.applyForce(tangent * ((ang_vel * c_p)))
    P_rotor.applyForce(Vecteur3D(-1,0,0) * (cart_velocity * c_r))

    if univers.manual_control:

        if not univers.started:
            if univers.gameKeys[ord('f')]:
                univers.started = True  
                univers.start_time = univers.time[-1]
        else:
            f_down.active= True
            if univers.gameKeys[ord('d')]:
                P_rotor.applyForce(Vecteur3D(3000,0,0))
            elif univers.gameKeys[ord('a')]:
                P_rotor.applyForce(Vecteur3D(-3000,0,0))
    else:

        if not univers.started:
            if univers.gameKeys[ord('f')]:
                univers.started = True
                univers.start_time = univers.time[-1]
        else:
            
            f_down.active= True


            # if angle <= 0 or abs(angle - (math.pi/2.)) > 1.0:
            #     gain_position = -0.25#0.15
            #     gain_angle = -5000
            #     output3 = (gain_position * univers.position_control.control(position_error, 0)) 

            #     output3 = min(max(output3, -30000),30000)
            #     output1 = (gain_angle * univers.angle_control.control(-math.pi/2., angle)) 

            #     output2 = 0

            #     if abs(angle + math.pi/2) < 0.1:
            #         output2 = 4000
            #     PDOutput = output1+output2+output3
            #     PDOutput = min(max(PDOutput, -10000), 10000)
            #     P_rotor.applyForce(Vecteur3D(PDOutput,0,0))
            energy = ((1/2. * ang_vel**2)/(m_p*L_p**2) + (1+math.sin(angle)))

            if choice == "3":
                univers.energy.append(energy)


            if angle <= 0 or abs(angle - (math.pi/2.)) > 1.0:
                if univers.gameKeys[ord('d')]:
                    P_pend.applyForce(Vecteur3D(400,0,0))
                elif univers.gameKeys[ord('a')]:
                    P_pend.applyForce(Vecteur3D(-400,0,0))

                gain_position = 0.025
                output3 = (gain_position * univers.position_control.control(position_error, 0)) 

                K = 100

                E_err = energy - 2


                force = - K * ang_vel *  math.sin(angle) * E_err

                force = min(max(force + output3, -10000), 10000)

                P_pend.applyForce(Vecteur3D(-force,0,0))
                

            else:
                
                if univers.gameKeys[ord('d')]:
                    P_pend.applyForce(Vecteur3D(400,0,0))
                elif univers.gameKeys[ord('a')]:
                    P_pend.applyForce(Vecteur3D(-400,0,0))
                

                gain_angle = -50000
                gain_position = 0.15 #0.15

                if abs(position_error) < 20:
                    univers.position_control.integral = 0

                output1 = (gain_angle * univers.angle_control.control(math.pi/2., angle)) 


                output3 = (gain_position * univers.position_control.control(position_error, 0)) 

                PDOutput = output1 + output3
                PDOutput = min(max(PDOutput, -20000), 20000)

                P_rotor.applyForce(Vecteur3D(PDOutput,0,0))   
                old_angle = angle 



if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 5',step=0.0001)
    sim_env.manual_control = False

    #Parametres du TP:

    m_r= 0.095 * 1e2
    L_r = 0.085 * 1e3

    c_r = 5e-4 * 1e3 #friction visqueuse f

    m_p = 0.024 * 1e2#Pendulum mass and length
    L_p = 0.129 * 1e3

    c_p = 5e-5 * 1e3


    while True:
        print("\n\n\n\n\n\n\n\n\n\nChoisissez la demo:    ")
        print(" 1. Controle manuel du rotor ")
        print(" 2. Stabilisation automatique, depart avec pendule deja inverse, angle 45")
        print(" 3. Stabilisation automatique, depart avec pendule pointant vers le bas ")

        
        choice = input("Choice ? : \n ")

        if choice == "1":
            break
        elif choice == "2":
            break
        elif choice == "3":
            break
        else:
            print("\n\n Choix incorrect.")
            print("Choix incorrect.")
            print("Choix incorrect.")


    L_p = 0.129 * 1e3 



    starting_angle = 45 #Degrees, clockwise
    
    if choice == "1":
        sim_env.manual_control = True
        P_rotor = Particule(p0=Vecteur3D(800,300,0),fix=False,name="Rotor", draw=False, mass=1, color="black", constrain=True)
        P_pend = Particule(p0=pendulum_coordinates(L_p,0), name="Pendule", color="red", mass=m_p/m_r, draw=False)
    elif choice == "2":
        P_rotor = Particule(p0=Vecteur3D(200,300,0),fix=False,name="Rotor", draw=False, mass=1, color="black", constrain=True)
        P_pend = Particule(p0=pendulum_coordinates(L_p,starting_angle), name="Pendule", color="red", mass=m_p/m_r, draw=False)
    elif choice == "3":
        P_rotor = Particule(p0=Vecteur3D(500,300,0),fix=False,name="Rotor", draw=False, mass=1, color="black", constrain=True)
        P_pend = Particule(p0=pendulum_coordinates(L_p,180), name="Pendule", color="red", mass=m_p/m_r, draw=False)


    print("\n\n\n\n\n\n\n\n\n")
    print("------------------------------------")
    print("")
    print("        Partie 3: Script 5          ")
    print("         Pendule inverse            ")
    print("                                    ")
    print("         Press F to start           ")
    print("        A,D to apply force          ")
    print("")
    print("------------------------------------")

    sim_env.angle = [math.pi*0.5 + math.radians(starting_angle)]
    sim_env.pos1 = [P_rotor.getPosition().x]
    sim_env.rise_time1 = None
    sim_env.rise_time2 = None
    sim_env.energy = [0]
    

    sim_env.addAgent(P_rotor, P_pend)

    f_link = Link(P_rotor, P_pend, k = 1e6)
    f_down = Gravity(Vecteur3D(0,-9807,0), active=False)
    f_damp = Damping(c=0.1)

    sim_env.addGenerators(f_link, f_down, f_damp)

    sim_env.started = False

    sim_env.angle_control = PIDController(1.0,0.07875,sim_env.step)
    sim_env.position_control = PIDController(60.0,50,sim_env.step, Ki = 15.0, windup_lim=400)


    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #

    # sim_env.plot()   
    from pylab import figure,plot,show,legend,xlabel,ylabel,title,ylim, xlim,axhline,grid,stem

    if choice == "2":
        print(f"ANGLE: Rise time: {sim_env.rise_time1 - sim_env.start_time:.3f}s, Settling time: {sim_env.settling_time1- sim_env.start_time:.3f}s")
        print(f"POS: Rise time: {sim_env.rise_time2 - sim_env.start_time:.3f}s, Settling time: {sim_env.settling_time2- sim_env.start_time:.3f}s")

    sim_env.angle = [i - math.pi*0.5 for i in sim_env.angle]

    figure("Pendule")
    if not choice == "1":
        title(f"Angle of the pendulum, Rise time: {sim_env.rise_time1 - sim_env.start_time:.3f}s, Settling time: {sim_env.settling_time1- sim_env.start_time:.3f}s ")
    else:
        title("Angle of the pendulum")
    plot(sim_env.time,sim_env.angle)
    xlabel("Time (s)")
    xlim(left=sim_env.start_time - 0.2)
    ylim(-math.pi - 0.2, math.pi + 0.2)
    axhline(0.0, color='r', linestyle='--', label="Target")
    ylabel("Angle (rad)")
    legend()
    show() 

    sim_env.pos1 = [(525 - i)*0.1  for i in sim_env.pos1]

    figure("Pendule")
    if choice =="2":
        title(f"Position of the cart, Rise time: {sim_env.rise_time2 - sim_env.start_time:.3f}s, Settling time: {sim_env.settling_time2- sim_env.start_time:.3f}s")
    elif choice == "3":
        title(f"Position of the cart, Settling time: {sim_env.settling_time2- sim_env.start_time:.3f}s")
    else:
        title(f"Position of the cart")
    plot(sim_env.time,sim_env.pos1)
    xlim(left= sim_env.start_time - 0.2)
    axhline(0.0, color='r', linestyle='--',  label="Target")
    legend()
    xlabel("Time (s)")
    ylabel("Position x (cm)")
    show() 

    if choice == "3":
        figure("Energy")
        title("Energy of the pendulum, normalized")
        plot(sim_env.time[-len(sim_env.energy):],sim_env.energy)
        xlabel("Time (s)")
        xlim(left=sim_env.start_time - 0.2)
        axhline(2, color='r', linestyle='--', label="Target energy")
        ylabel("Energy (normalized)")
        legend()
        show() 
