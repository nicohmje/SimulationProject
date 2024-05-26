from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
    Masse + (ressort+amortisseur)

    avec application au clavier d'une force constante ou harmonique.
"""

def customEvents(univers:Univers):
    univers.pos.append(P_mass.getPosition().x)

    if univers.gameKeys[ord('f')] and not univers.force_active :
        if univers.choice == "1":
            P_mass.applyForce(Vecteur3D(500,0,0))
        else:   
            P_mass.applyForce(Vecteur3D(500 *math.cos(univers.time[-1]* (math.pi/2.))))
            univers.force_active = True
    elif univers.force_active and univers.choice == "2":
            P_mass.applyForce(Vecteur3D(500 *math.cos(univers.time[-1]* (math.pi/2.))))
    

if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 1',step=0.001)

    P_fixe = Particule(p0=Vecteur3D(200,250,0),fix=True,name="Base",color="white")
    P_mass = Particule(p0=Vecteur3D(600,250,0),name="Mass",color="cyan", mass=1)


    sim_env.addAgent(P_fixe,P_mass)
    sim_env.pos = [600]


    f_down = Gravity(Vecteur3D(0,-10,0),active=False)

    axis = Vecteur3D(0,1,0)

    while True:
        print("\n\n\n\n\n\n\nChoisissez le type de systeme:    ")
        print(" 1. Systeme sous-amorti ")
        print(" 2. Systeme sur-amorti ")
        print(" 3. Systeme criticalement amorti ")

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

    choice1 = choice

    while True:
        print("\n\n\n\n\n\n\nChoisissez la force a appliquer:    ")
        print(" 1. Force constante ")
        print(" 2. Force harmonique (Cosinus de période 4s)")
        
        choice = input("Choice ? : \n ")

        if choice == "1":
            break
        elif choice == "2":
            break
        else:
            print("\n\n Choix incorrect.")
            print("Choix incorrect.")

    print("\n\n\n\n\n\n\n\n\n")
    print("------------------------------------")
    print("")
    print("        Partie 3: Script 1          ")
    print("    masse + (ressort+amortisseur)   ")
    print("                                    ")
    if choice == "1":
        print("       Hold F to apply force       ")
    else:
        print("      Press F to start force       ")
    print("")
    print("------------------------------------")
    print("\n\n\n")

    sim_env.choice = choice
    sim_env.force_active = False

    k=10
    match choice1:
        case "1":
            c = c=(0.1,1)[choice =="2"]
        case "2":
            c = 12
        case "3":
            c = math.sqrt(4 * k * P_mass.mass)

    # c=(0.1,1)[choice =="2"] #C = sqrt(4mk)
    # c=(100,1)[choice =="2"] #C = sqrt(4mk)
    f_link = Prismatic(P_fixe,P_mass, axis, k=k, c=c)

    sim_env.addGenerators(f_link)

    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #

    from pylab import figure,plot,show,legend,xlabel,ylabel,title

    sim_env.pos = sim_env.pos - np.mean(sim_env.pos)
    t = np.array(sim_env.time)

    zero_crossings = np.where(np.diff(np.signbit(sim_env.pos)))[0]
    average_period = 2*np.mean(np.diff(t[zero_crossings]))

    w = np.sqrt(k / P_mass.mass)

    f = (w/(2*math.pi), 0.25)[choice=="2"]

    print(f"Theoreritical frequency: {f}Hz")
    print(f"Measured frequency:  {1/average_period}Hz")
    

    figure("Spring Mass Damper ")
    if choice == "1":
        match choice1:
            case "1":
                title(f"Mass-Spring-Damper f_theo: {f:.3f}hz, f_mesu: {1/average_period:.3f}Hz")
            case "2":
                title(f"Overdamped Mass-Spring-Damper, c={c:.3f}")
            case "3":
                title(f"Critically damped MSD")
    else:
        match choice1:
            case "1":
                title(f"Mass-Spring-Damper f_theo: {f:.3f}hz, f_mesu: {1/average_period:.3f}Hz")
            case "2":
                title(f"Overdamped Mass-Spring-Damper, c={c:.3f},  f_theo: {f:.3f}hz, f_mesu: {1/average_period:.3f}Hz")
            case "3":
                title(f"Critically damped MSD,  f_theo: {f:.3f}hz, f_mesu: {1/average_period:.3f}Hz")
    plot(sim_env.time,sim_env.pos,label="P_mass position")
    xlabel("Time (s)")
    ylabel("Position (m)")
    legend()
    show()
