from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
    Masse + (ressort+amortisseur)

    avec application au clavier d'une force constante ou harmonique.
"""

def resetToPos(pos:Vecteur3D, particule:Particule):

    particule.position.append(pos)
    particule.acceleration.append(Vecteur3D())
    particule.speed.append(Vecteur3D())

def pendulum_coordinates(L, theta):
    # Convert theta from degrees to radians
    origin = P_fixe.getPosition()

    theta_radians = math.radians(theta)
    
    # Calculate the coordinates of the pendulum bob
    x = origin.x + L * math.sin(theta_radians)
    y = origin.y + L * math.cos(theta_radians)
    
    return Vecteur3D(int(x), int(y),0)

def customEvents(univers:Univers):
    if f_down.active:
        univers.pos.append(pends[0].getPosition().x)
    if univers.gameKeys[ord('f')]:
        if not f_down.active:
            univers.start = len(univers.time)
            f_down.active = True
    elif univers.gameKeys[ord('g')]:
        for i,pend in enumerate(pends):
            resetToPos(pendulum_coordinates(pendulums[i], angle), pend)
        univers.pos = [pends[0].getPosition().x]
        f_down.active = False
    pass

def HzToLength(f):

    L = (1/(f*2*math.pi))**2 * 9.81

    return L


if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 2',step=0.005)

    P_fixe = Particule(p0=Vecteur3D(500,450,0),fix=True,name="Base",color="white")

    while True:
        print("\n\n\n\n\n\n\n\n\n\nChoisissez la demo:    ")
        print(" 1. Trois pendules de 10,20,30 cm ")
        print(" 2. Pendulum waves (la video youtube) ")
        
        choice = input("Choice ? : \n ")

        if choice == "1":
            break
        elif choice == "2":
            break
        else:
            print("\n\n Choix incorrect.")
            print("Choix incorrect.")
            print("Choix incorrect.")


    if choice == "2":

        pendulums = [HzToLength(j)*1000 for j in [((i+50)/60.) for i in range(1,15,1)]]

        f_down = Gravity(Vecteur3D(0,-9810,0),active=False)
        f_damp = Damping(0.05, "air friction?", active=True)

        print("\n\n\n\n\n\n\n\n\n")
        print("------------------------------------")
        print("")
        print("        Partie 3: Script 2          ")
        print("          Pendulum waves            ")
        print("                                    ")
        print("         Press F to start           ")
        print("         Press G to reset           ")
        print("")
        print("------------------------------------")
        print("\n\n\n")

    else:

        pendulums = [100,200,300]
        f_down = Gravity(Vecteur3D(0,-9810,0),active=False)
        f_damp = Damping(0.1, "air friction?", active=True)

        print("\n\n\n\n\n\n\n\n\n")
        print("------------------------------------")
        print("")
        print("        Partie 3: Script 2          ")
        print("    Trois pendules, L= 10,20,30cm   ")
        print("                                    ")
        print("         Press F to start           ")
        print("         Press G to reset           ")
        print("")
        print("------------------------------------")
        print("\n\n\n")

    

    angle = -135 #degrees 

    pends = []

    for i in pendulums: 
        pends.append(Particule(p0=pendulum_coordinates(i, angle), name=f"pend_{(i*0.1):.1f}", color="cyan", mass=1, draw=False))


    sim_env.addAgent(P_fixe,*pends)
    sim_env.pos = [pends[0].getPosition().x]
    sim_env.index = 0




    

    links = []
    dampers = []

    for i in pends:
        links.append(Link(P_fixe, i))



    sim_env.addGenerators(f_down, *links, f_damp)



    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #
    sim_env.plot()

    from pylab import figure,plot,show,legend,xlabel,ylabel,title

    if choice == "1":
        sim_env.pos = sim_env.pos - np.mean(sim_env.pos)
        t = np.array(sim_env.time[sim_env.start:])

        zero_crossings = np.where(np.diff(np.signbit(sim_env.pos)))[0]
        average_period = 2*np.mean(np.diff(t[zero_crossings]))
        figure("Pendule")
        title(f"Pendule de {pendulums[0]*0.1} cm, periode: {average_period:.3f}s")
        plot(t,sim_env.pos)
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()
    
    
