# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 17:13:08 2024

@author: UserTP
"""

from vecteur3D import Vecteur3D
from math import pi,atan2
import pygame
from pygame.locals import *
from time import time

class Particule(object):

    def __init__(self, mass=1, p0=Vecteur3D(), v0=Vecteur3D(), a0=Vecteur3D(), fix=False, name="paf", color='red', draw=True, constrain=False):
        self.mass = mass
        self.position = [p0]
        self.speed = [v0]
        self.acceleration = [a0]
        self.name = name
        self.color = color
        self.forces = Vecteur3D()
        self.fix = fix
        self.draw = draw
        self.constrain = constrain

    def __str__(self):
        msg = 'Particule ('+str(self.mass)+', '+str(self.position[-1])+', '+str(self.speed[-1])+', '+str(self.acceleration[-1])+', "'+self.name+'", "'+str(self.color)+'" )'
        return msg

    def __repr__(self):
        return str(self)

    def applyForce(self, *args):
        for f in args:
            self.forces += f

    def simule(self,step):
        self.pfd(step)
        
    def pfd(self, step):
        if not(self.fix):
            a = self.forces * (1/self.mass)
            v = self.speed[-1]+a*step
            if self.constrain:
                a.y = 0
                v.y = 0
        else :
            a = Vecteur3D()
            v = Vecteur3D()

        p = self.position[-1]+0.5*a*step**2 + self.speed[-1]*step

        self.acceleration.append(a)
        self.speed.append(v)
        self.position.append(p)
        self.forces = Vecteur3D()

    def plot(self):
        from pylab import plot
        X=[]
        Y=[]
        for p in self.position:
            X.append(p.x)
            Y.append(p.y)
    
        return plot(X,Y,color=self.color,label=self.name)+plot(X[-1],Y[-1],'o',color=self.color)    

    def getPosition(self):
        return self.position[-1]
    
    def getSpeed(self):
        return self.speed[-1]
    
    def gameDraw(self,scale,screen):
    
        W , H =  screen.get_size()     
        X = int(scale*self.getPosition().x)
        Y = int(scale*self.getPosition().y)
        Z = int(scale*self.getPosition().z)
        
        vit = self.getSpeed()
        VX = int(scale*vit.x)
        VY = int(scale*vit.y) 
        VZ = int(scale*vit.z) 
        
        size=2
        
        if type(self.color) is tuple:
            color = (self.color[0]*255,self.color[1]*255,self.color[2]*255)
        else:
            color=self.color
        Xp = X+Z*(2**0.5)/2
        Yp = (H-Y) - Z*(2**0.5)/2
        VXp = Xp + VX + VZ*(2**0.5)/2
        VYp = H - ((Y +Z*(2**0.5)/2) +(VY+VZ*(2**0.5)/2))

        pygame.draw.circle(screen,color,(Xp,Yp),size*2,size)
        if self.draw:
            pygame.draw.line(screen,color,(Xp,Yp),(VXp,VYp))
    

class Univers(object):
    
    def __init__(self,t0=0,step=0.1,name="plage",population=[],dimensions=(1024,512)):
        
        self.name=name
        self.time=[t0]
        self.population=population
        self.step=step
        
        self.dimensions=dimensions
        
        self.mouseControlled = []        
        self.keyControlled = []
        
        self.generators = []

       
        
    def __str__(self):
        ret = "Univers (" + str(self.time[-1]) + ","  + str(self.step) + ", " + self.name + ", " + str(self.population) + ")"
        return ret
    
    def __repr__(self):
        return str(self)
    
    def addAgent(self,*args):
        for agent in args:
            self.population.append(agent)
            agent.time = self.time
            
        return len(self.population)
    
    def addGenerators(self,*args):
        for g in args:
            self.generators.append(g)
            
    def simulate(self):
        for agent in self.population :
            agent.simule(self.step)
        
        self.time.append(self.time[-1]+self.step)
    
    def simulateTo(self,tFin):
        while self.time[-1]<tFin:
            self.simulate()
     
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()
        
    def gameInit(self,scale=1,fps=60,background=(0,0,0)):
        
        """- initialiser pygame
        - dessiner la fenetre
        """
        
        pygame.init()
        self.t0= time()
        
        self.clock = pygame.time.Clock()
        self.background=background
        self.fps=fps
        self.scale=scale
        self.running=True
        
        self.W = self.dimensions[0]*self.scale
        self.H = self.dimensions[1]*self.scale
        
        self.targetX = 0
        self.targetY = 0
       
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption(self.name)
        
        
        
    
    def gameUpdate(self):
               
        # Set real world time
        self.now = time()-self.t0
        
        # Check for user interaction
        self.gameInteraction()
        
        
        # run physics simulation steps for this frame
        while self.time[-1] < self.now:
            # Check for user-defined events in MAIN
            try:
               from __main__ import customEvents
               customEvents(self)
            except Exception as error:
               print("An error occurred:", error)
            
             
            for p in self.population:
                for generator in self.generators:
                    generator.setForce(p)
            
            
            self.simulate()
        
        # Draw everything
        self.screen.fill(self.background)
        
        font_obj = pygame.font.Font('freesansbold.ttf', 12)
        text_surface_obj = font_obj.render(('time: %.2f' % self.now), True, 'green', self.background)
        text_rect_obj = text_surface_obj.get_rect()
        text_rect_obj.topleft = (0, 0)
        
        self.screen.blit(text_surface_obj, text_rect_obj)

        for agent in self.population:
            agent.gameDraw(self.scale,self.screen)
            

        pygame.display.update()
        
        # Wait for the next frame
        self.clock.tick(self.fps)
        
    def gameInteraction(self):
        pygame.event.pump() # process event queue
        
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: # window is closed ?
                self.running = False

        self.gameKeys = pygame.key.get_pressed() # It gets the states of all keyboard keys.
        self.gameMouseClick = pygame.mouse.get_pressed() # mouse buttons (list)
        self.gameMousePos = pygame.mouse.get_pos()       # mouse position (in pixels)

        
        if self.now > self.gameEndTime or self.gameKeys[K_ESCAPE]: # quit if ESC is pressed
            self.running=False        
        
        
        if self.gameMouseClick[0]:  # mouse click position in game dimensions 
            self.targetX = self.gameMousePos[0] / self.scale
            self.targetY = (self.H - self.gameMousePos[1]) / self.scale
            
        for generator in self.mouseControlled:
            pass
            
        for generator in self.keyControlled:
            generator.active = False
            if self.gameKeys[ord(' ')] or self.gameKeys[pygame.K_SPACE]: # And if the key is K_DOWN:
                generator.active =not(generator.active) 
                                      
            if self.gameKeys[ord('z')] or self.gameKeys[pygame.K_UP]: # And if the key is K_DOWN:
                pass
            if self.gameKeys[ord('s')] or self.gameKeys[pygame.K_DOWN]: # And if the key is K_DOWN:
                pass
            if self.gameKeys[ord('q')] or self.gameKeys[pygame.K_LEFT]: # And if the key is K_DOWN:
                pass
            if self.gameKeys[ord('d')] or self.gameKeys[pygame.K_RIGHT]: # And if the key is K_DOWN:
                pass
  
    def simulateRealTime(self,scale=1,fps=60,background=(0,0,0),tfin=50000000):

        self.gameEndTime = tfin
        self.gameInit(scale,fps,background)
     
        while self.running:
            self.gameUpdate()
                        
        pygame.quit()   
        
class Force(object):
    
    def __init__(self,force=Vecteur3D(),name='force',active=True):
        self.force = force
        self.name = name
        self.active = active
        
    def __str__(self):
        return "Force ("+str(self.force)+', '+self.name+")"
        
    def __repr__(self):
        return str(self)

    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.force)
    
class ForceSelect(Force):
    
    def __init__(self,force=Vecteur3D(),name='force',particules=[],active=True):
        Force.__init__(self,force,name,active)
        self.particules=particules
    
    def setForce(self, particule):
        if self.active and particule in self.particules:
            particule.applyForce(self.force)

class Gravity(Force):
    def __init__(self,force=Vecteur3D(0,-9.81,0),name='gravity',active=True):
        Force.__init__(self,force,name,active)

    def setForce(self, particule):
        if self.active :
            # print("GRAVITY: ", self.force * particule.mass)
            particule.applyForce(self.force*particule.mass)

class Damping(Force):
    def __init__(self,c=0,name='damp',active=True):
        Force.__init__(self,Vecteur3D(),name,active)
        self.c=c
        
    def setForce(self, particule):
        if self.active :
            particule.applyForce(-self.c*particule.getSpeed())
        

class SpringDamper(Force):
    def __init__(self,P0,P1,k=0,c=0,l0=0,active=True,name="boing"):
        Force.__init__(self,Vecteur3D(),name,active)
        self.k = k
        self.c = c
        self.P0 = P0
        self.P1 = P1
        self.l0 = l0
    
    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        v_n = vec_dir.norm()
        flex = vec_dir.mod()-self.l0
        
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c 
        
        force = (self.k * flex + vit_n)* v_n
        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)
        else:
            pass
        
class Link(SpringDamper):
    def __init__(self,P0,P1, k=1000, c=100):
        l0 = (P0.getPosition()-P1.getPosition()).mod()
        SpringDamper.__init__(self,P0, P1,k,c,l0,True,"link")

class Prismatic(SpringDamper):
    def __init__(self,P0,P1,axis=Vecteur3D(1,0,0), k=1000, c=100):
        self.axis = axis.norm()
        l = (P0.getPosition()-P1.getPosition())
        l0 = l-(l**axis) * axis
        SpringDamper.__init__(self,P0, P1,k,c,l0.mod(),True,"link")
        
    
    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        vec_dir_proj = vec_dir - (vec_dir ** self.axis) * self.axis
        v_n = vec_dir_proj.norm()
        
        flex = vec_dir_proj.mod()-self.l0
        
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c 
        
        force = (self.k * flex + vit_n)* v_n
        
        
        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)
        else:
            pass


class MoteurCC:
    def __init__(self, p1:Particule, p2:Particule, res = 1, inductance = 0.001, kc = 0.01, ke = 0.01, rotorJ = 0.01, resistive_torque=0, externJ=0, max_voltage=0, f=0.1):

        ## MOTOR PARAMS
        self.resistance_Ohm = res
        self.inductance_H = inductance
        self.backemf = 0

        self.max_voltage_V = max_voltage

        self.torque_constant_Nm_A = kc
        self.backemf_constant_Vs = ke

        self.rotor_inertia_kgm2 = rotorJ
        self.total_inertia_kgm2 = rotorJ + externJ

        self.viscous_friction_Nms = f

        self.resistive_torque_Nm = resistive_torque

        self.voltage_V = [0]
        self.current_A = [0] 
        self.torque_Nm = [0]

    def __str__(self):
        return "Moteur CC (Speed : "+str(self.speed_rad_s[-1])+', Torque : '+str(self.torque_Nm[-1])+', Voltage '+self.voltage_V[-1]+', Current '+str(self.current_A[-1])+')'

    def __repr__(self):
        return str(self)
    
    def setExternTorque(self, extern_torque):
        self.resistive_torque_Nm = extern_torque
        print("ext torque: ", extern_torque)
    
    def setInertia(self, load_inertia):
        
        """
            set inertia (J)
        """
        self.total_inertia_kgm2 = self.rotor_inertia_kgm2 + load_inertia

        denom = 1 / (self.torque_constant_Nm_A * self.backemf_constant_Vs + self.resistance_Ohm * self.viscous_friction_Nms)
        self.tau = (self.resistance_Ohm * self.total_inertia_kgm2) * denom
        self.K = self.torque_constant_Nm_A * denom

    def setViscosity(self, visco):
        self.viscous_friction_Nms = visco

    
    def setVoltage(self, voltage):
        if self.max_voltage_V:
            voltage = min(self.max_voltage_V, voltage)
        self.voltage_V.append(voltage)

    def getCurrent(self):
        return self.current_A[-1]
    
    def getSpeed(self):
        return self.speed_rad_s[-1]

    def getTorque(self):
        return self.torque_Nm[-1]
    
    def getPosition(self):
        return self.position_rad[-1]
    
    def getAllStates(self):
        return self.position_rad, self.speed_rad_s, self.torque_Nm, self.current_A
    
    
    def d_dt(self,current,speed):
        
        """
            Derivatives for current and speed (see formula given by S. Haliyo)
        """

        didt = (self.voltage_V[-1] - self.resistance_Ohm * current - self.backemf_constant_Vs * speed) / self.inductance_H
        dodt = (self.torque_constant_Nm_A * current - self.viscous_friction_Nms * speed) / self.total_inertia_kgm2
        
        # print(current,speed,dodt,didt)
        return [dodt, didt]

    def rk4(self, current, speed):

        """
            Parallel RK4 to numerically integrate the DC motor equation.
        """

        #the o suffix is for omega, as in the speed
        #the i suffix is for the current
        derivatives = self.d_dt(current, speed)
        k1_o = derivatives[0] * self.stepsize_s
        k1_i = derivatives[1] * self.stepsize_s

        derivatives = self.d_dt(current + 0.5*k1_i, speed + 0.5*k1_o)
        k2_o = self.stepsize_s * derivatives[0]
        k2_i = self.stepsize_s * derivatives[1]

        derivatives = self.d_dt(current + 0.5*k2_i, speed + 0.5*k2_o)
        k3_o = self.stepsize_s * derivatives[0]
        k3_i = self.stepsize_s * derivatives[1]

        derivatives = self.d_dt(current+ k3_i, speed+k3_o)
        k4_o = self.stepsize_s * derivatives[0]
        k4_i = self.stepsize_s * derivatives[1]

        new_speed = speed + 1/6. * (k1_o + 2*k2_o + 2*k3_o + k4_o)
        new_current = current + 1/6. * (k1_i + 2*k2_i + 2*k3_i + k4_i)
        # time.sleep(3)
        return new_current, new_speed
    

    def simule(self, step):

        """
            Simulate a step. Numerically integrate current and speed, then integrate speed to get position. 
        """

        self.stepsize_s = step
        new_curr, new_speed = self.rk4(self.current_A[-1], self.speed_rad_s[-1])
        # print(new_curr, new_speed)
        # time.sleep(1)
        
        self.current_A.append(new_curr)
        self.speed_rad_s.append(new_speed)
        self.torque_Nm.append(self.torque_constant_Nm_A * new_curr - self.resistive_torque_Nm)
        self.position_rad.append(self.position_rad[-1] + new_speed*self.stepsize_s)

    
if __name__=='__main__':
    from pylab import figure, show, legend

    P0 =Particule(v0=Vecteur3D(10,10,0))
    print(P0)
    
    while P0.getPosition().y >= 0. :
        P0.applyForce(Vecteur3D(0, -9.81, 0))
        P0.pfd(0.01)
        
    figure()
    P0.plot()
    legend()
    show()
    
    