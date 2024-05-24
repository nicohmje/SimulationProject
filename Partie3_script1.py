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
            P_mass.applyForce(Vecteur3D(250 *math.cos(univers.time[-1]* (math.pi/2.))))
            univers.force_active = True
    elif univers.force_active and univers.choice == "2":
            P_mass.applyForce(Vecteur3D(250 *math.cos(univers.time[-1]* (math.pi/2.))))
    

def compute_fft(time_series, sampling_rate):
    """
    Computes the FFT of a given time series and sampling rate, returns frequencies and magnitudes in dB,
    and excludes the zero frequency component.
    
    Args:
    - time_series (np.array): The data points of the time series.
    - sampling_rate (float): The sampling rate of the data (number of samples per second).
    
    Returns:
    - frequencies (np.array): Array of frequencies corresponding to the FFT output.
    - magnitudes_dB (np.array): Magnitude of the FFT at each frequency in decibels.
    """
    n = len(time_series)
    fft_result = np.fft.fft(time_series)
    frequencies = np.fft.fftfreq(n, d=1/sampling_rate)
    magnitudes = np.abs(fft_result)
    
    # Convert magnitudes to dB
    # magnitudes_dB = 20 * np.log10(magnitudes)
    
    # Exclude zero frequency (first element) for both frequencies and magnitudes
    return frequencies[1:n//2], magnitudes[1:n//2]  # Return only the positive half of the spectrum, excluding zero

if __name__ == "__main__":
    sim_env = Univers(name='Part 3 Script 1',step=0.001)

    P_fixe = Particule(p0=Vecteur3D(200,250,0),fix=True,name="Base",color="white")
    P_mass = Particule(p0=Vecteur3D(600,250,0),name="Mass",color="cyan", mass=1)


    sim_env.addAgent(P_fixe,P_mass)
    sim_env.pos = [600]


    f_down = Gravity(Vecteur3D(0,-10,0),active=False)

    axis = Vecteur3D(0,1,0)

    while True:
        print("\n\n\n\n\n\n\nChoisissez la force a appliquer:    ")
        print(" 1. Force constante ")
        print(" 2. Force harmonique ")
        
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
    print("       Press F to apply force       ")
    print("")
    print("------------------------------------")
    print("\n\n\n")

    sim_env.choice = choice
    sim_env.force_active = False

    k=10
    c=(0.1,1)[choice =="2"]
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
    title(f"Mass-Spring-Damper f_theo: {f:.3f}hz, f_mesu: {1/average_period:.3f}Hz")
    plot(sim_env.time,sim_env.pos,label="P_mass position")
    xlabel("Time (s)")
    ylabel("Position (m)")
    legend()
    show()
