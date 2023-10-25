#Currently working

import os, sys, math, pickle, time
from zmqRemoteApi import RemoteAPIClient
from random import randint as ri
from random import uniform as ru
import pandas as pd

cluster = False
if cluster == True:
    port_conexion = int(sys.argv[1])
    print ('Base Port:', port_conexion)
# Whether the CESGA is not used, select the port '23000' by default.
else:
    port_conexion = 23000

#Class for simulation object
class sim_objects ():
    def __init__(self):
        self.client = 0
        self.sim = 0
        self.file = 0
        self.joint_handler_ids = []
        self.num_joints = 0
        self.obj_handler_ids = []
        
        #Number of sequences and steps per sequence
        self.sequencies = 500
        self.seq_steps = 100
        #self.sequencies = 1000
        #self.seq_steps = 10

        #Stores all the training data to be saved
        self.training_data = []

        #Stores initial perceptions measured in the load function
        self.initial_j_positions = []
        self.initial_j_velocities = []
        self.initial_j_forces = []
        self.initial_base_pos_x = 0
        self.initial_base_pos_y = 0
        self.initial_base_pos_z = 0
        self.initial_base_ori_alpha = 0
        self.initial_base_ori_beta = 0
        self.initial_base_ori_gamma = 0
        
#sim_obj = sim_objects()


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
        
#handler = joint_handlers_class()

class obj_handlers_class ():
    def __init__(self):
        self.JRF2 = 0
        self.JRB2 = 0
        self.JLF2 = 0
        self.JLB2 = 0
        self.base = 0

#obj_handler = obj_handlers_class()


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

#perception = perceptions()


# Function that load the robot in the scene at the beggining of each evaluation.
def load_quad_class():
    # Opening the connection with CoppeliaSim simulator
    sim_obj.client = RemoteAPIClient('localhost', port_conexion)
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


def init_position():
    start_pos = ri(0,1)
    #Fixes the last joints position to 0 so they wont move
    for obj in sim_obj.obj_handler_ids[:-1]:
        sim_obj.sim.setJointTargetPosition(obj, 0)
    if start_pos == 0:
        #Set all the joints to their position
        for joint_init in sim_obj.joint_handler_ids:
            sim_obj.sim.setJointTargetPosition(joint_init, 0)
        #After setting all the joints it moves    
        for t_step_i in range(20):
            sim_obj.client.step()
    else:
        sim_obj.client.step()
    #Starting initial perceptions
    sim_obj.initial_j_positions = []
    sim_obj.initial_j_velocities = []
    sim_obj.initial_j_forces = []
    #Get initial perceptions
    base_x, base_y, base_z = sim_obj.sim.getObjectPosition(obj_handler.base, sim_obj.sim.handle_world)
    base_alpha, base_beta, base_gamma = sim_obj.sim.getObjectOrientation(obj_handler.base, sim_obj.sim.handle_world)
    sim_obj.initial_base_pos_x = base_x
    sim_obj.initial_base_pos_x = base_y
    sim_obj.initial_base_pos_x = base_z
    sim_obj.initial_base_ori_alpha = base_alpha
    sim_obj.initial_base_ori_beta = base_beta
    sim_obj.initial_base_ori_gamma = base_gamma    
    #Joint perceptions
    for joint_i_per in sim_obj.joint_handler_ids:
        #Angular/linear position of the joint
        pos = sim_obj.sim.getJointPosition(joint_i_per)
        #Torque/force being aplied by the joint
        force = sim_obj.sim.getJointForce(joint_i_per)
        #Angular/linear velocity of the joint's movement
        veloc = sim_obj.sim.getJointVelocity(joint_i_per)
        sim_obj.initial_j_positions.append(pos)
        sim_obj.initial_j_velocities.append(veloc)
        sim_obj.initial_j_forces.append(force)


#Generate list of random positions
def rand_gen():
    for num in range(sim_obj.num_joints):
        n = ru(-7.5, 7.5) * math.pi / 180
        perception.increments.append(n)


#Function to move all the joints to a random position
def move_joints():
    if perception.step != 0:
        for joint_num in range(sim_obj.num_joints):
            prev_pos = sim_obj.training_data[-1]['post_j_positions']
            sim_obj.sim.setJointTargetPosition(sim_obj.joint_handler_ids[joint_num], (perception.increments[joint_num]+prev_pos[joint_num]))
        #After setting all the joints it moves
        for t_step in range(5):
            sim_obj.client.step()
    else:
        for joint_num in range(sim_obj.num_joints):
            sim_obj.sim.setJointTargetPosition(sim_obj.joint_handler_ids[joint_num], (perception.increments[joint_num]+sim_obj.initial_j_positions[joint_num]))
        #After setting all the joints it moves
        for t_step in range(5):
            sim_obj.client.step()


def get_preceptions():
    #Define previous perceptions
    if perception.step != 0:
        perception.prev_base_pos_x = sim_obj.training_data[-1]['post_base_pos_x']
        perception.prev_base_pos_y = sim_obj.training_data[-1]['post_base_pos_y']
        perception.prev_base_pos_z = sim_obj.training_data[-1]['post_base_pos_z']
        perception.prev_base_ori_alpha = sim_obj.training_data[-1]['post_base_ori_alpha']
        perception.prev_base_ori_beta = sim_obj.training_data[-1]['post_base_ori_beta']
        perception.prev_base_ori_gamma = sim_obj.training_data[-1]['post_base_ori_gamma']
        perception.prev_j_positions = sim_obj.training_data[-1]['post_j_positions']
        perception.prev_j_velocities = sim_obj.training_data[-1]['post_j_velocities']
        perception.prev_j_forces = sim_obj.training_data[-1]['post_j_forces']    
    else:
        perception.prev_base_pos_x = sim_obj.initial_base_pos_x 
        perception.prev_base_pos_y = sim_obj.initial_base_pos_y
        perception.prev_base_pos_z = sim_obj.initial_base_pos_z
        perception.prev_base_ori_alpha = sim_obj.initial_base_ori_alpha
        perception.prev_base_ori_beta = sim_obj.initial_base_ori_beta
        perception.prev_base_ori_gamma = sim_obj.initial_base_ori_gamma
        perception.prev_j_positions = sim_obj.initial_j_positions
        perception.prev_j_velocities = sim_obj.initial_j_velocities
        perception.prev_j_forces = sim_obj.initial_j_forces 

    #Get perceptions
    base_x, base_y, base_z = sim_obj.sim.getObjectPosition(obj_handler.base, sim_obj.sim.handle_world)
    base_alpha, base_beta, base_gamma = sim_obj.sim.getObjectOrientation(obj_handler.base, sim_obj.sim.handle_world)
    perception.post_base_pos_x = base_x
    perception.post_base_pos_y = base_y
    perception.post_base_pos_z = base_z
    perception.post_base_ori_alpha = base_alpha
    perception.post_base_ori_beta = base_beta
    perception.post_base_ori_gamma = base_gamma
        
    #Joint perceptions
    for joint_per in sim_obj.joint_handler_ids:
        #Angular/linear position of the joint
        pos = sim_obj.sim.getJointPosition(joint_per)
        #Torque/force being aplied by the joint
        force = sim_obj.sim.getJointForce(joint_per)
        #Angular/linear velocity of the joint's movement
        veloc = sim_obj.sim.getJointVelocity(joint_per)
        perception.post_j_positions.append(pos)
        perception.post_j_velocities.append(veloc)
        perception.post_j_forces.append(force)


def export():    
    #Dump data in file
    timestr = time.strftime("_%Y_%d%m_")
    sim_obj.file = open('training_dataset_' + str(port_conexion) + timestr + '.pkl', 'wb')
    pickle.dump(sim_obj.training_data, sim_obj.file)
    sim_obj.file.close()
    if not cluster:
        df = pd.DataFrame(sim_obj.training_data)
        df.to_csv('data_' + str(port_conexion) + timestr + '.csv', index=False)


#Main function for multiple executions and multiple steps
def main():
    #Inits classes to be used
    global sim_obj
    sim_obj = sim_objects()
    global handler
    handler = joint_handlers_class()
    global obj_handler
    obj_handler = obj_handlers_class()
    global perception
    perception = perceptions()
    #Starts sequencies
    for seq in range(sim_obj.sequencies):
        load_quad_class()
        init_position()
        print('seq:', seq)
        for seq_step in range(sim_obj.seq_steps):
            perception = perceptions()
            perception.sequence = seq
            perception.step = seq_step    
            rand_gen()
            #Execute every joint action
            move_joints()
            get_preceptions()
            sim_obj.training_data.append(vars(perception))
        sim_obj.sim.stopSimulation()    
        export()
    

if __name__ == "__main__":
    main()