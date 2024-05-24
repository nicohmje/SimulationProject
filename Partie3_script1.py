from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
    Masse + (ressort+amortisseur)

    avec application au clavier d'une force constante ou harmonique.
"""

def customEvents(univers:Univers):

    univers.pos.append(P_mass.getPosition().x)

    if univers.gameKeys[ord('f')]:
        P_mass.applyForce(Vecteur3D(500,0,0))
    elif univers.gameKeys[ord('g')]:
        
        P_mass.applyForce(Vecteur3D(math.pi * 500 *math.sin(univers.index)))
        univers.index += univers.step
        pass 
    else:
        univers.index = 0

    pass


if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 1',step=0.001)

    P_fixe = Particule(p0=Vecteur3D(200,250,0),fix=True,name="Base",color="white")
    P_mass = Particule(p0=Vecteur3D(600,250,0),name="Mass",color="cyan", mass=1)


    sim_env.addAgent(P_fixe,P_mass)
    sim_env.pos = [600]
    sim_env.index = 0


    f_down = Gravity(Vecteur3D(0,-10,0),active=False)

    axis = Vecteur3D(0,1,0)

    print("\n\n\n\n\n\n\n\n\n")
    print("------------------------------------")
    print("")
    print("        Partie 3: Script 1          ")
    print("    masse + (ressort+amortisseur)   ")
    print("                                    ")
    print("        Press F to apply 500N       ")
    print("      Press G to apply 500N sine    ")
    print("")
    print("------------------------------------")
    print("\n\n\n")

    k=10
    c=1
    f_link = Prismatic(P_fixe,P_mass, axis, k=k, c=c)


    sim_env.addGenerators(f_link)

    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #

    from pylab import figure,plot,show,legend,xlabel,ylabel,title

    figure()
    title(f"Systeme Masse+ (ressort-amortisseur) (Force 500N, k={k}, c={c})")
    plot(sim_env.time,sim_env.pos,label="P_mass position")
    xlabel("Time (s)")
    ylabel("Position (cm)")
    legend()
    show()
    
    
