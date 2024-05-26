from particule import * 
from Partie1 import * 
from vecteur3D import *


"""
    Masse + (ressort+amortisseur)

    avec application au clavier d'une force constante ou harmonique.
"""

def compute_fft(time_series, step):
    n = len(time_series)
    fft_result = np.fft.fft(time_series)
    frequencies = np.fft.fftfreq(n, d=step)
    magnitudes = np.abs(fft_result)

    magnitudes = 20 * np.log10(magnitudes) #dB 
    
    # Exclude zero frequency (first element) for both frequencies and magnitudes
    return frequencies[1:n//2], magnitudes[1:n//2]  # Return only the positive half of the spectrum, excluding zero


def customEvents(univers:Univers):
    if univers.choice == "1":
        univers.dist1.append(100 - (P_1.getPosition() - P_fixe.getPosition()).x)
        univers.dist2.append(100 - (P_2.getPosition() - P_1.getPosition()).x)
        univers.dist3.append(100 -(P_fixe2.getPosition() - P_2.getPosition()).x)
        univers.pos2.append(P_2.getPosition().x)
        univers.pos1.append(P_1.getPosition().x)
        force = 100000
    elif univers.choice == "2":
        univers.pos3.append(P_3.getPosition().x)
        univers.pos2.append(P_2.getPosition().x)
        univers.pos1.append(P_1.getPosition().x)
        force = 100000
    else: 
        univers.pos4.append(P_4.getPosition().x)
        univers.pos3.append(P_3.getPosition().x)
        univers.pos2.append(P_2.getPosition().x)
        univers.pos1.append(P_1.getPosition().x)
        force = 100000
    
    if univers.gameKeys[ord('f')] and not univers.started:
        univers.started = True

        match univers.choice:

            case "1": # 2 ddl
                if univers.choice2 == "2":
                    pends[0].applyForce(Vecteur3D(force,0,0))
                    pends[1].applyForce(Vecteur3D(force,0,0))
                elif univers.choice2 == "1":
                    pends[0].applyForce(Vecteur3D(-force,0,0))
                    pends[1].applyForce(Vecteur3D(force,0,0))
                elif univers.choice2 == "0":
                    pends[0].applyForce(Vecteur3D(-0.5 * force,0,0))
                    pends[1].applyForce(Vecteur3D(0.2*force,0,0))
                    



            case "2": # 3 ddl

                match univers.choice2: 
                    case "0":
                        pends[0].applyForce(Vecteur3D(0.2*force,0,0))
                        pends[1].applyForce(Vecteur3D(0.5*force * math.sqrt(2),0,0))
                        pends[2].applyForce(Vecteur3D(-force,0,0))
                    case "1":
                        pends[0].applyForce(Vecteur3D(force,0,0))
                        pends[1].applyForce(Vecteur3D(force * math.sqrt(2),0,0))
                        pends[2].applyForce(Vecteur3D(force,0,0))
                    case "2":
                        pends[0].applyForce(Vecteur3D(force,0,0))
                        pends[2].applyForce(Vecteur3D(-force,0,0))
                    case "3":
                        pends[0].applyForce(Vecteur3D(force,0,0))
                        pends[1].applyForce(Vecteur3D(-force* math.sqrt(2),0,0))
                        pends[2].applyForce(Vecteur3D(force,0,0))

            case "3": # 4 ddl 

                match univers.choice2: 
                    case "0":
                        pends[0].applyForce(Vecteur3D(0.5*force,0,0))
                        pends[1].applyForce(Vecteur3D(-force,0,0))
                        pends[2].applyForce(Vecteur3D(0.8*force,0,0))
                        pends[3].applyForce(Vecteur3D(0.2 * force,0,0))
                    case "1":
                        pends[0].applyForce(Vecteur3D(force,0,0))
                        pends[1].applyForce(Vecteur3D(1.618 * force,0,0))
                        pends[2].applyForce(Vecteur3D(1.618 * force,0,0))
                        pends[3].applyForce(Vecteur3D(force,0,0))
                    case "2":
                        pends[0].applyForce(Vecteur3D(-1.618 * force,0,0))
                        pends[1].applyForce(Vecteur3D(-force,0,0))
                        pends[2].applyForce(Vecteur3D(force,0,0))
                        pends[3].applyForce(Vecteur3D(1.618 * force,0,0))
                    case "3":
                        pends[0].applyForce(Vecteur3D(1.618*force,0,0))
                        pends[1].applyForce(Vecteur3D(-force,0,0))
                        pends[2].applyForce(Vecteur3D(-force,0,0))
                        pends[3].applyForce(Vecteur3D(1.618*force,0,0))
                    case "4":
                        pends[0].applyForce(Vecteur3D(force,0,0))
                        pends[1].applyForce(Vecteur3D(-1.618*force,0,0))
                        pends[2].applyForce(Vecteur3D(1.618*force,0,0))
                        pends[3].applyForce(Vecteur3D(-force,0,0))


if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 4',step=0.001)

    sim_env.index = 0

    while True:
        print("\n\n\n\n\n\n\n\n\n\nChoisissez la demo:    ")
        print(" 1. Systeme masse-ressort 2 DDL ")
        print(" 2. Systeme masse-ressort 3 DDL ")
        print(" 3. Systeme masse-ressort 4 DDL ")
        
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
    

    while True:
        print("\n\n\n\n\n\n\n\n\n\nChoisissez votre mode propre:    ")
        print(" 0. Chaos ")
        match choice:
            case "1":
                print(" 1. Opposition de phase | - +")
                print(" 2. En phase | + +")
            case "2":
                print(" 1. Les trois masses, meme signe | + ++ + ")
                print(" 2. Deux masses exterieures, signe oppose | + x - ")
                print(" 3. Les trois masses, milieu oppose | + -- + ")
            case "3":
                print(" 1. Paire gauche, paire droite | + ++ ++ + ")
                print(" 2. Paire gauche, paire droite, signes opposes | -- - + ++ ")
                print(" 3. Paire exterieure, paire interieure | ++ - - ++ ")
                print(" 4. Paires ABAB | + -- ++ - ")
        
        choice2 = input("Choice ? : \n ")

        if choice2 == "0":
            break
        elif choice2 == "1":
            break
        elif choice2 == "2":
            break
        elif choice2 == "3" and int(choice)>1:
            break
        elif choice2 == "4" and int(choice)>2:
            break
        else:
            print("\n\n Choix incorrect.")
            print("Choix incorrect.")
            print("Choix incorrect.")

    sim_env.choice = choice
    sim_env.choice2 = choice2

    match choice:
        case "1":
            P_fixe = Particule(p0=Vecteur3D(300,300,0),fix=True,name="Base",color="white")
            P_fixe2 = Particule(p0=Vecteur3D(600,300,0),fix=True,name="Base",color="white")
            P_1 = Particule(p0=Vecteur3D(400, 300, 0), name=f"m1", color="yellow", mass=1, draw=False)
            P_2 = Particule(p0=Vecteur3D(500, 300, 0), name=f"m2", color="green", mass=1, draw=False)
            pends=[P_1, P_2]
            sim_env.addAgent(P_fixe, P_fixe2, P_1, P_2)
        case "2":
            P_fixe = Particule(p0=Vecteur3D(250,300,0),fix=True,name="Base",color="white")
            P_fixe2 = Particule(p0=Vecteur3D(650,300,0),fix=True,name="Base",color="white")
            P_1 = Particule(p0=Vecteur3D(350, 300, 0), name=f"m1", color="yellow", mass=1, draw=False)
            P_2 = Particule(p0=Vecteur3D(450, 300, 0), name=f"m2", color="green", mass=1, draw=False)
            P_3 = Particule(p0=Vecteur3D(550, 300, 0), name=f"m3", color="blue", mass=1, draw=False)
            pends=[P_1, P_2, P_3]
            sim_env.addAgent(P_fixe, P_fixe2, P_1, P_2, P_3)
        case "3":
            P_fixe = Particule(p0=Vecteur3D(200,300,0),fix=True,name="Base",color="white")
            P_fixe2 = Particule(p0=Vecteur3D(700,300,0),fix=True,name="Base",color="white")
            P_1 = Particule(p0=Vecteur3D(300, 300, 0), name=f"m1", color="yellow", mass=1, draw=False)
            P_2 = Particule(p0=Vecteur3D(400, 300, 0), name=f"m2", color="green", mass=1, draw=False)
            P_3 = Particule(p0=Vecteur3D(500, 300, 0), name=f"m3", color="blue", mass=1, draw=False)
            P_4 = Particule(p0=Vecteur3D(600, 300, 0), name=f"m4", color="red", mass=1, draw=False)
            pends=[P_1, P_2, P_3, P_4]
            sim_env.addAgent(P_fixe, P_fixe2, P_1, P_2, P_3, P_4)


    # f_down = Gravity(Vecteur3D(0,-9810,0),active=False)
    f_damp = Damping(0.05, "air friction?", active=True)
    
    try:
        sim_env.pos4 = [P_4.getPosition().x]
    except:
        pass
    try:
        sim_env.pos3 = [P_3.getPosition().x]
    except:
        pass

    sim_env.pos2 = [P_2.getPosition().x]
    sim_env.pos1 = [P_1.getPosition().x]

    print("\n\n\n\n\n\n\n\n\n")
    print("------------------------------------")
    print("")
    print("        Partie 3: Script 4          ")
    print(" Systeme masse ressort {choice}DDL  ")
    print("                                    ")
    print("       Press F to apply force        ")
    print("")
    print("------------------------------------")
    print("\n\n\n")


    k = 100
    c = 0
    axis = Vecteur3D(0,1,0)


    links = [Prismatic(P_fixe,pends[0], axis, k=k, c=c)]
    for i in range(0,int(choice)):
        links.append(Prismatic(pends[i],pends[i+1], axis, k=k, c=c))
    links.append(Prismatic(pends[-1],P_fixe2, axis, k=k, c=c))

    if choice == "1":
        sim_env.dist1 = [100 - (P_1.getPosition() - P_fixe.getPosition()).x]
        sim_env.dist2 = [100 - (P_2.getPosition() - P_1.getPosition()).x]
        sim_env.dist3 = [100 - (P_fixe2.getPosition() - P_2.getPosition()).x]

    sim_env.addGenerators(*links, f_damp)
    sim_env.started = False


    # Start simulation #####################################################
    sim_env.simulateRealTime(scale=1,background=(30,30,30))

    ###############################
    #
    # Post-traitement
    #

    from pylab import figure,plot,show,legend,xlabel,ylabel,title,ylim, xlim,axvline,grid,stem
    
    if choice == "1":
        sim_env.pos1 = sim_env.pos1 - np.mean(sim_env.pos1)

        frequencies, magnitudes_dB = compute_fft(sim_env.pos1, sim_env.step)

        f_theo1 = (1. / (2 * math.pi)) * math.sqrt(k/P_1.mass)
        f_theo2 = f_theo1 * math.sqrt(3)

        print(f"Frequences theoriques: f1={f_theo1:.3f}, f2={f_theo2:.3f}")

        figure("FFT")
        stem(frequencies, magnitudes_dB, 'b', markerfmt=" ", basefmt="-b")
        title(f'FFT f_theo1={f_theo1:.3f}Hz, f_theo2={f_theo2:.3f}Hz')
        axvline(x=f_theo1, color='r', linestyle='--', label='F_theo1')
        axvline(x=f_theo2, color='r', linestyle='--', label='F_theo2')
        xlabel('Frequence (Hz)')
        ylabel('Magnitude (dB)')
        xlim(0, 10)  # Limit to 10 Hz for better visibility
        ylim(bottom=0)
        legend()
        grid(True)
        show()

        t = np.array(sim_env.time)

        figure("Masse 2")
        title(f"Masse 2")
        plot(t,sim_env.pos1)
        xlabel("Time (s)")
        ylabel("Position x (m)")
        show()

        figure("Ressort2")
        title(f"Ressort central")
        plot(t,sim_env.dist2)
        ylim(-10,10)
        xlabel("Time (s)")
        ylabel("Elongation (m)")
        show()

        sim_env.pos2 = sim_env.pos2 - np.mean(sim_env.pos2)

        zero_crossings = np.where(np.diff(np.signbit(sim_env.pos2)))[0]
        average_period = 2*np.mean(np.diff(t[zero_crossings]))
        figure("Masse 2")
        title(f"Masse 2")
        plot(t,sim_env.pos2)
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        show()

        figure("Masses")
        title(f"Deplacement des masses")
        plot(t,sim_env.pos1, label="Masse 1")
        plot(t,sim_env.pos2, label="Masse 2")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()    
    elif choice == "2":

        t = np.array(sim_env.time)

        sim_env.pos1 = sim_env.pos1 - np.mean(sim_env.pos1)

        frequencies, magnitudes_dB = compute_fft(sim_env.pos1, sim_env.step)

        w =  math.sqrt(k/P_1.mass)
        f_theo1 = 1/(math.pi * 2) * math.sqrt(w**2 * (2 - math.sqrt(2)))
        f_theo2 = 1/(math.pi * 2) * math.sqrt(w**2 * 2)
        f_theo3 = 1/(math.pi * 2) * math.sqrt(w**2 * (2 + math.sqrt(2)))

        print(f"Frequences theoriques: f1={f_theo1:.3f}, f2={f_theo2:.3f},  f3={f_theo3:.3f}")

        figure("FFT")
        stem(frequencies, magnitudes_dB, 'b', markerfmt=" ", basefmt="-b")
        title(f'FFT f_theo1={f_theo1:.3f}Hz, f_theo2={f_theo2:.3f}Hz, f_theo3={f_theo3:.3f}Hz' )
        axvline(x=f_theo1, color='r', linestyle='--', label='F_theo1')
        axvline(x=f_theo2, color='r', linestyle='--', label='F_theo2')
        axvline(x=f_theo3, color='r', linestyle='--', label='F_theo3')
        xlabel('Frequence (Hz)')
        ylabel('Magnitude (dB)')
        xlim(0, 10)  # Limit to 10 Hz for better visibility
        legend()
        grid(True)
        show()

        sim_env.pos1 = sim_env.pos1 - np.mean(sim_env.pos1)
        sim_env.pos2 = sim_env.pos2 - np.mean(sim_env.pos2)
        sim_env.pos3 = sim_env.pos3 - np.mean(sim_env.pos3)


        figure("Masses")
        title(f"Masses 1,2,3")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos2, label="m2")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 1,2")
        plot(t,sim_env.pos2, label="m2")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 2,3")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos2, label="m2")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 1,3")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()
        pass

    elif choice == "3":

        t = np.array(sim_env.time)

        sim_env.pos1 = sim_env.pos1 - np.mean(sim_env.pos1)

        frequencies, magnitudes_dB = compute_fft(sim_env.pos1, sim_env.step)

        f =  1/(math.pi*2) * math.sqrt(k/P_1.mass)
        f_theo1 = 0.618 * f
        f_theo2 = 1.176 * f 
        f_theo3 = 1.618 * f
        f_theo4 = 1.902 * f


        print(f"Frequences theoriques: f1={f_theo1:.3f}, f2={f_theo2:.3f},  f3={f_theo3:.3f}, f4={f_theo4:.3f}")

        figure("FFT")
        stem(frequencies, magnitudes_dB, 'b', markerfmt=" ", basefmt="-b")
        title(f'FFT f_1={f_theo1:.3f}Hz, f_2={f_theo2:.3f}Hz, f_3={f_theo3:.3f}Hz, f_4={f_theo4:.3f}Hz')
        axvline(x=f_theo1, color='r', linestyle='--', label='F_theo1')
        axvline(x=f_theo2, color='r', linestyle='--', label='F_theo2')
        axvline(x=f_theo3, color='r', linestyle='--', label='F_theo3')
        axvline(x=f_theo4, color='r', linestyle='--', label='F_theo4')
        xlabel('Frequence (Hz)')
        ylabel('Magnitude (dB)')
        xlim(0, 10)  # Limit to 10 Hz for better visibility
        legend()
        grid(True)
        show()

        sim_env.pos1 = sim_env.pos1 - np.mean(sim_env.pos1)
        sim_env.pos2 = sim_env.pos2 - np.mean(sim_env.pos2)
        sim_env.pos3 = sim_env.pos3 - np.mean(sim_env.pos3)
        sim_env.pos4 = sim_env.pos4 - np.mean(sim_env.pos4)


        figure("Masses")
        title(f"Masses 1,2,3,4")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos4, label="m4")
        plot(t,sim_env.pos2, label="m2")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 1,4")
        plot(t,sim_env.pos4, label="m2")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 2,3")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos2, label="m2")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 1,2")
        plot(t,sim_env.pos2, label="m3")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 3,4")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos4, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 1,3")
        plot(t,sim_env.pos3, label="m3")
        plot(t,sim_env.pos1, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()

        figure("Masses")
        title(f"Masses 2,4")
        plot(t,sim_env.pos2, label="m3")
        plot(t,sim_env.pos4, label="m1")
        xlabel("Time (s)")
        ylabel("Position x (cm)")
        legend()
        show()
        pass
