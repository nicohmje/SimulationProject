from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
    Masse + (ressort+amortisseur)

    avec application au clavier d'une force constante ou harmonique.
"""
def customEvents(univers:Univers):
    sim_env.pos2.append(P_2.getPosition().x)
    sim_env.pos1.append(P_1.getPosition().x)
    if univers.gameKeys[ord('f')]:
        P_1.applyForce(Vecteur3D(500,0,0))
    pass


if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 4',step=0.0001)

    P_fixe = Particule(p0=Vecteur3D(300,300,0),fix=True,name="Base",color="white")
    P_fixe2 = Particule(p0=Vecteur3D(600,300,0),fix=True,name="Base",color="white")

    # f_down = Gravity(Vecteur3D(0,-9810,0),active=False)
    f_damp = Damping(0.05, "air friction?", active=True)

    P_1 = Particule(p0=Vecteur3D(400, 300, 0), name=f"m1", color="yellow", mass=1, draw=False)
    P_2 = Particule(p0=Vecteur3D(500, 300, 0), name=f"m2", color="green", mass=1, draw=False)

    sim_env.addAgent(P_fixe, P_fixe2, P_1, P_2)
    sim_env.pos2 = [P_2.getPosition().x]
    sim_env.pos1 = [P_1.getPosition().x]
    sim_env.index = 0

    k = 100
    c = 0


    axis = Vecteur3D(0,1,0)
    f_link1 = Prismatic(P_fixe,P_1, axis, k=k, c=c)
    f_link2 = Prismatic(P_1,P_2, axis, k=k, c=c)
    f_link3 = Prismatic(P_2,P_fixe2, axis, k=k, c=c)

    sim_env.addGenerators(f_link1, f_link2, f_link3, f_damp)


    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #

    sim_env.plot()

    from pylab import figure,plot,show,legend,xlabel,ylabel,title


    sim_env.pos1 = sim_env.pos1 - np.mean(sim_env.pos1)
    t = np.array(sim_env.time)

    zero_crossings = np.where(np.diff(np.signbit(sim_env.pos1)))[0]
    average_period = 2*np.mean(np.diff(t[zero_crossings]))
    figure("Masse1")
    title(f"Masse1, periode: {average_period:.3f}s")
    plot(t,sim_env.pos1)
    xlabel("Time (s)")
    ylabel("Position x (cm)")
    legend()
    show()

    sim_env.pos2 = sim_env.pos2 - np.mean(sim_env.pos2)

    zero_crossings = np.where(np.diff(np.signbit(sim_env.pos2)))[0]
    average_period = 2*np.mean(np.diff(t[zero_crossings]))
    figure("Masse1")
    title(f"Masse1, periode: {average_period:.3f}s")
    plot(t,sim_env.pos2)
    xlabel("Time (s)")
    ylabel("Position x (cm)")
    legend()
    show()
    
