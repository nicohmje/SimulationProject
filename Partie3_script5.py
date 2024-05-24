from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
   Pendule inverse

    controle manuel et automatique
"""


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
    if univers.manual_control:

        if not univers.started:
            if univers.gameKeys[ord('f')]:
                univers.started = True  
        else:
            f_down.active= True
            if univers.gameKeys[ord('d')]:
                P_rotor.applyForce(Vecteur3D(1500,0,0))
            elif univers.gameKeys[ord('a')]:
                P_rotor.applyForce(Vecteur3D(-1500,0,0))
    else:

        if not univers.started:
            if univers.gameKeys[ord('f')]:
                univers.started = True
        else:
            
            f_down.active= True


            axis = (P_rotor.getPosition() - P_pend.getPosition())
            global old_angle
            global angle
            old_angle = angle
            angle = -math.atan2(axis.y, axis.x)

            if angle <= 0 or abs(angle - (math.pi/2.)) > 1.0:
                gain_position = -0.25#0.15
                gain_angle = -100
                position_error = P_rotor.getPosition().x - 525
                output3 = (gain_position * univers.position_control.control(position_error, 0)) 

                output3 = min(max(output3, -30000),30000)
                output1 = (gain_angle * univers.angle_control.control(-math.pi/2., angle)) 

                output2 = 0

                if abs(angle + math.pi/2) < 0.1:
                    output2 = 4000
                PDOutput = output1+output2+output3
                PDOutput = min(max(PDOutput, -10000), 10000)
                P_rotor.applyForce(Vecteur3D(PDOutput,0,0))
                

            else:

                ang_vel = (old_angle-angle)/univers.step
                
                if univers.gameKeys[ord('d')]:
                    P_pend.applyForce(Vecteur3D(400,0,0))
                elif univers.gameKeys[ord('a')]:
                    P_pend.applyForce(Vecteur3D(-400,0,0))
                elif univers.gameKeys[ord('l')]:
                    print("jdwadw")
                    P_rotor.applyForce(Vecteur3D(100,0,0))
                elif univers.gameKeys[ord('j')]:
                    print("jdwadw")
                    P_rotor.applyForce(Vecteur3D(-100,0,0))


                kart_velocity = P_rotor.getSpeed().x
                mass_velocity = kart_velocity + ang_vel * math.sin(angle)

                position_error = P_rotor.getPosition().x - 525

                gain_angle = -500
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

    while True:
        print("\n\n\n\n\n\n\n\n\n\nChoisissez la demo:    ")
        print(" 1. Controle manuel du rotor ")
        print(" 2. Stabilisation automatique, depart avec pendule deja inverse ")
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


    
    if choice == "1":
        sim_env.manual_control = True
        P_rotor = Particule(p0=Vecteur3D(800,300,0),fix=False,name="Rotor", draw=False, mass=1, color="black", constrain=True)
        P_pend = Particule(p0=Vecteur3D(750,350, 0), name="Pendule", color="red", mass=0.5, draw=False)
    elif choice == "2":
        P_rotor = Particule(p0=Vecteur3D(500,300,0),fix=False,name="Rotor", draw=False, mass=1, color="black", constrain=True)
        P_pend = Particule(p0=Vecteur3D(430,350, 0), name="Pendule", color="red", mass=0.5, draw=False)
    elif choice == "3":
        P_rotor = Particule(p0=Vecteur3D(500,300,0),fix=False,name="Rotor", draw=False, mass=1, color="black", constrain=True)
        P_pend = Particule(p0=Vecteur3D(500,200, 0), name="Pendule", color="red", mass=0.5, draw=False)


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
    

    sim_env.addAgent(P_rotor, P_pend)

    f_link = Link(P_rotor, P_pend)
    f_down = Gravity(Vecteur3D(0,-1000,0), active=False)

    sim_env.addGenerators(f_link, f_down)

    sim_env.started = False

    sim_env.angle_control = PIDController(50,25,sim_env.step)
    sim_env.position_control = PIDController(4.0,250,sim_env.step, Ki = 2.0, windup_lim=4000)


    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #

    # sim_env.plot()    
