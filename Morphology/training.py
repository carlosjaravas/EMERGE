import os
import math
import pickle
from zmqRemoteApi import RemoteAPIClient
from random import randint as ri
from random import uniform as ru
import pandas as pd

#Class for simulation object
class sim_objects ():
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.file = 0
        self.executed = False
        self.joint_handler_ids = []
        self.num_joints = 0
        self.obj_handler_ids = []
        self.sequencies = 1
        self.seq_steps = 2
        self.training_data = []
        
sim_obj = sim_objects()

# Class to store the different handlers the robot needs to operate in the simulator. 
class joint_handlers_class ():
    def __init__(self):
        self.JRF0 = 0
        self.JRF1 = 0
        self.JRB0 = 0
        self.JRB1 = 0
        self.JLF0 = 0
        self.JLF1 = 0
        self.JLB0 = 0
        self.JLB1 = 0
        
handler = joint_handlers_class()

class obj_handlers_class ():
    def __init__(self):
        self.JRF2 = 0
        self.JRB2 = 0
        self.JLF2 = 0
        self.JLB2 = 0
        self.base = 0

obj_handler = obj_handlers_class()

#Funtion to randomize de initial position
def init_position():
    start_pos = ri(0,1)
    #Fixes the last joints position to 0 so they wont move
    for i in sim_obj.obj_handler_ids[:-1]:
        sim_obj.sim.setJointTargetPosition(i, 0)
    if start_pos == 0:
        #Set all the joints to their position
        for i in sim_obj.joint_handler_ids:
            sim_obj.sim.setJointTargetPosition(i, 0)
        #After setting all the joints it moves    
        for i in range(20):
            sim_obj.client.step()
    else:
        sim_obj.client.step()

# Function that load the robot in the scene at the beggining of each evaluation.
def load_quad_class():
    # Opening the connection with CoppeliaSim simulator
    sim_obj.client = RemoteAPIClient()
    sim_obj.sim = sim_obj.client.getObject('sim')
    
    # Setting the simulation mode for working in sinchronous mode. The simulation timming is controlled by the Python program and not by the simulation itselfs.
    sim_obj.client.setStepping(True)
    
    #Right-front
    handler.JRF0 = sim_obj.sim.getObject("/JRF0")
    handler.JRF1 = sim_obj.sim.getObject("/JRF1")
    #Right-back
    handler.JRB0 = sim_obj.sim.getObject("/JRB0")
    handler.JRB1 = sim_obj.sim.getObject("/JRB1")
    #Left-front
    handler.JLF0 = sim_obj.sim.getObject("/JLF0")
    handler.JLF1 = sim_obj.sim.getObject("/JLF1")
    #Left-back
    handler.JLB0 = sim_obj.sim.getObject("/JLB0")
    handler.JLB1 = sim_obj.sim.getObject("/JLB1")

    #Setting obj handlers
    obj_handler.base = sim_obj.sim.getObject("/Base")
    obj_handler.JRF2 = sim_obj.sim.getObject("/JRF2")
    obj_handler.JRB2 = sim_obj.sim.getObject("/JRB2")
    obj_handler.JLF2 = sim_obj.sim.getObject("/JLF2")
    obj_handler.JLB2 = sim_obj.sim.getObject("/JLB2")

    
    #Stores the different id for every handler starting with the base
    sim_obj.joint_handler_ids = list(handler.__dict__.values())

    sim_obj.num_joints = len(sim_obj.joint_handler_ids)

    sim_obj.obj_handler_ids = list(obj_handler.__dict__.values())


    sim_obj.sim.startSimulation()

#Class to store all the perceptions
class perceptions ():
    def __init__(self):
        #Execution data
        self.sequence = 0
        self.step = 0
        self.increments = []

        #Initial perceptions
        self.prev_j_positions = []
        self.prev_j_velocities = []
        self.prev_j_forces = []
        self.prev_base_pos_x = 0
        self.prev_base_pos_y = 0
        self.prev_base_pos_z = 0
        self.prev_base_ori_alpha = 0
        self.prev_base_ori_beta = 0
        self.prev_base_ori_gamma = 0
        #self.prev_base_accel = []

        #Resulting perceptions
        self.post_j_positions = []
        self.post_j_velocities = []
        self.post_j_forces = []
        self.post_base_pos_x = 0
        self.post_base_pos_y = 0
        self.post_base_pos_z = 0
        self.post_base_ori_alpha = 0
        self.post_base_ori_beta = 0
        self.post_base_ori_gamma = 0
        #self.post_base_accel = []

perception = perceptions()

def get_joints_preceptions():

    #Base perceptions
    base_x, base_y, base_z = sim_obj.sim.getObjectPosition(obj_handler.base, sim_obj.sim.handle_world)
    base_alpha, base_beta, base_gamma = sim_obj.sim.getObjectOrientation(obj_handler.base, sim_obj.sim.handle_world)
    if  not sim_obj.executed:
        perception.prev_base_pos_x = base_x
        perception.prev_base_pos_y = base_y
        perception.prev_base_pos_z = base_z
        perception.prev_base_ori_alpha = base_alpha
        perception.prev_base_ori_beta = base_beta
        perception.prev_base_ori_gamma = base_gamma
    else:
        perception.post_base_pos_x = base_x
        perception.post_base_pos_y = base_y
        perception.post_base_pos_z = base_z
        perception.post_base_ori_alpha = base_alpha
        perception.post_base_ori_beta = base_beta
        perception.post_base_ori_gamma = base_gamma
        
    #Joint perceptions
    for i in sim_obj.joint_handler_ids:
        #Angular/linear position of the joint
        pos = sim_obj.sim.getJointPosition(i)
        #Torque/force being aplied by the joint
        force = sim_obj.sim.getJointForce(i)
        #Angular/linear velocity of the joint's movement
        veloc = sim_obj.sim.getJointVelocity(i)
        if  not sim_obj.executed:
            perception.prev_j_positions.append(pos)
            perception.prev_j_velocities.append(veloc)
            perception.prev_j_forces.append(force)
        else:
            perception.post_j_positions.append(pos)
            perception.post_j_velocities.append(veloc)
            perception.post_j_forces.append(force)

#Function to execute one increment in every joint
def move_joints():
    for i in range(sim_obj.num_joints):
        sim_obj.sim.setJointTargetPosition(sim_obj.joint_handler_ids[i], (perception.increments[i]+perception.prev_j_positions[i]))
    #After setting all the joints it moves
    for i in range(5):
        sim_obj.client.step()

#Generate list of random positions
def rand_gen():
    for i in range(sim_obj.num_joints):
        n = ru(-10, 10) * math.pi / 180
        perception.increments.append(n)

#Main function for multiple executions and multiple steps
sim_obj.training_data = []
for k in range(sim_obj.sequencies):
    load_quad_class()
    init_position()
    for j in range(sim_obj.seq_steps):
        perception = perceptions()
        perception.sequence = k
        perception.step = j         
        rand_gen()
        get_joints_preceptions()
        #Execute every joint action
        move_joints()
        sim_obj.executed = True
        get_joints_preceptions()
        sim_obj.executed = False
        sim_obj.training_data.append(vars(perception))
    sim_obj.sim.stopSimulation()


df = pd.DataFrame(sim_obj.training_data)
df.to_csv("data.csv", index=False)   