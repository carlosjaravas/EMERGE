import os
import math
import pickle
from zmqRemoteApi import RemoteAPIClient
from random import uniform as ru

#Class for simulation object
class sim_objects ():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.file = 0
        self.random_increments = []
        self.executed = False
        self.handler_ids = []
        self.sequencies = 1
        self.seq_steps = 2

sim_obj = sim_objects()

# Class to store the different handlers the robot needs to operate in the simulator. 
class handlers_class ():
    def __init__(self):
        self.JRF0 = 0
        self.JRF1 = 0
        #self.JRF2 = 0
        self.JRB0 = 0
        self.JRB1 = 0
        #self.JRB2 = 0
        self.JLF0 = 0
        self.JLF1 = 0
        #self.JLF2 = 0
        self.JLB0 = 0
        self.JLB1 = 0
        #self.JLB2 = 0
        self.base = 0

handler = handlers_class()

# Function that load the robot in the scene at the beggining of each evaluation.
def load_quad_class():
    # Opening the connection with CoppeliaSim simulator
    sim_obj.client = RemoteAPIClient()
    sim_obj.sim = sim_obj.client.getObject('sim')
    
    # Setting the simulation mode for working in sinchronous mode. The simulation 
    # timming is controlled by the Python program and not by the simulation itselfs.
    sim_obj.client.setStepping(True)
    
    handler.base = sim_obj.sim.getObject("/Base")
    #Right-front
    handler.JRF0 = sim_obj.sim.getObject("/JRF0")
    handler.JRF1 = sim_obj.sim.getObject("/JRF1")
    #handler.JRF2 = sim_obj.sim.getObject("/JRF2")
    #Right-back
    handler.JRB0 = sim_obj.sim.getObject("/JRB0")
    handler.JRB1 = sim_obj.sim.getObject("/JRB1")
    #handler.JRB2 = sim_obj.sim.getObject("/JRB2")
    #Left-front
    handler.JLF0 = sim_obj.sim.getObject("/JLF0")
    handler.JLF1 = sim_obj.sim.getObject("/JLF1")
    #handler.JLF2 = sim_obj.sim.getObject("/JLF2")
    #Left-back
    handler.JLB0 = sim_obj.sim.getObject("/JLB0")
    handler.JLB1 = sim_obj.sim.getObject("/JLB1")
    #handler.JLB2 = sim_obj.sim.getObject("/JLB2")
    
    #Stores the different id for every handler starting with the base
    sim_obj.handler_ids = list(handler.__dict__.values())

    sim_obj.sim.startSimulation()

#Generate list of random positions
def rand_gen():
    randomlist = []
    for i in range(len(sim_obj.handler_ids)-1):
        n = ru(-10, 10) * math.pi / 180
        randomlist.append(n)
    sim_obj.random_increments.append(randomlist)

#Class to store all the perceptions
class perceptions ():
    def __init__(self):
        #Initial perceptions
        self.prev_j_positions = []
        self.prev_j_velocities = []
        self.prev_j_l_velocities = []
        self.prev_j_forces = []
        self.prev_base_pos = []
        self.prev_base_ori = []
        self.prev_base_accel = []

        #Resulting perceptions
        self.post_j_positions = []
        self.post_j_velocities = []
        self.post_j_l_velocities = []
        self.post_j_forces = []
        self.post_base_pos = []
        self.post_base_ori = []
        self.post_base_accel = []        

perception = perceptions()

def get_joints_preceptions():
    positions = []
    forces = []
    velocities = []
    l_velocities = []
    base_pos = sim_obj.sim.getObjectPosition(handler.base, sim_obj.sim.handle_world)
    base_ori = sim_obj.sim.getObjectOrientation(handler.base, sim_obj.sim.handle_world)
    base_accel = [0,0,0] #sim_obj.sim.getObjectOrientation(handler.base, sim_obj.sim.handle_world) --- MISSING
    for i in sim_obj.handler_ids[:-1]:
        #Angular/linear position of the joint
        pos = sim_obj.sim.getJointPosition(i)
        positions.append(pos)
        #Torque/force being aplied by the joint
        force = sim_obj.sim.getJointForce(i)
        forces.append(force)
        #Angular/linear velocity of the joint's movement
        veloc = sim_obj.sim.getJointVelocity(i)
        velocities.append(veloc)
        #Linear velocity of the joint
        l_veloc = sim_obj.sim.getJointVelocity(i)
        l_velocities.append(l_veloc)
    if  not sim_obj.executed:
        print("Getting prev perceptions")
        perception.prev_j_positions.append(positions)
        perception.prev_j_velocities.append(velocities)
        perception.prev_j_l_velocities.append(l_velocities)
        perception.prev_j_forces.append(forces)
        perception.prev_base_pos.append(base_pos)
        perception.prev_base_ori.append(base_ori)
        perception.prev_base_accel.append(base_accel)
    else:
        print("Getting post perceptions")
        perception.post_j_positions.append(positions)
        perception.post_j_velocities.append(velocities)
        perception.post_j_l_velocities.append(l_velocities)
        perception.post_j_forces.append(forces)
        perception.post_base_pos.append(base_pos)
        perception.post_base_ori.append(base_ori)
        perception.post_base_accel.append(base_accel)

def export():    
    #Dump data in file
    sim_obj.file = open(os.getcwd() + r'\training_dataset.txt', 'wb')
    pickle.dump(perception, sim_obj.file)
    sim_obj.file.close()

#Main function for multiple executions and multiple steps
def main():
    print("main function")
    for k in range(sim_obj.sequencies):
        print(f"{k} sequence has started")
        print("")
        #sim_obj.sim.startSimulation()
        load_quad_class()
        for j in range(sim_obj.seq_steps):
            print("----------- Secuencia: ", k)
            print("----------- Paso: ", j)
            rand_gen()
            get_joints_preceptions()
            #Execute every joint action
            print("Executing actions")
            for i in range(len(sim_obj.random_increments[-1])):
                sim_obj.sim.setJointTargetPosition(sim_obj.handler_ids[i], (sim_obj.random_increments[-1][i]+perception.prev_j_positions[-1][i]))
                sim_obj.client.step()
                sim_obj.client.step()
                sim_obj.client.step()
                sim_obj.client.step()
            sim_obj.executed = True
            get_joints_preceptions()
            sim_obj.executed = False
            print("")
        print(f"{k} simulation has finished")
        print("")
        sim_obj.sim.stopSimulation()
    export()

if __name__ == "__main__":
    main()