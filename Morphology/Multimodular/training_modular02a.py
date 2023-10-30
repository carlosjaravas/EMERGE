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

scene = "modular02a"

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
        self.sequencies = 20
        self.seq_steps = 100

        #Stores all the training data to be saved
        self.training_data = []

        #Stores initial perceptions measured in the load function
        self.initial_j_positions = []
        self.initial_pos_x = 0
        self.initial_pos_y = 0
        self.initial_pos_z = 0
        

# Class to store the different handlers the robot needs to operate in the simulator. 
class joint_handlers_class ():
    def __init__(self):
        self.J0 = 0
        self.J1 = 0
        

class obj_handlers_class ():
    def __init__(self):
        self.C1a = 0
        


#Class to store all the perceptions
class perceptions ():
    def __init__(self):
        #Execution data
        self.sequence = 0
        self.step = 0
        self.increments = []
        #Initial perceptions
        self.prev_j_positions = []
        self.prev_pos_x = 0
        self.prev_pos_y = 0
        self.prev_pos_z = 0
        #Resulting perceptions
        self.post_j_positions = []
        self.post_pos_x = 0
        self.post_pos_y = 0
        self.post_pos_z = 0
        #Change in positions
        self.delta_pos_x = 0
        self.delta_pos_y = 0
        self.delta_pos_z = 0


# Function that load the robot in the scene at the beggining of each evaluation.
def load_quad_class():
    # Opening the connection with CoppeliaSim simulator
    sim_obj.client = RemoteAPIClient('localhost', port_conexion)
    sim_obj.sim = sim_obj.client.getObject('sim')
    # Setting the simulation mode for working in sinchronous mode. The simulation timming is controlled by the Python program and not by the simulation itselfs.
    sim_obj.client.setStepping(True)
    #Joints
    handler.J0 = sim_obj.sim.getObject("/J0")
    handler.J1 = sim_obj.sim.getObject("/J1")

    #Setting obj handlers
    obj_handler.C1a = sim_obj.sim.getObject("/C1a")
 
    #Stores the different id for every handler starting with the base
    sim_obj.joint_handler_ids = list(handler.__dict__.values())

    sim_obj.num_joints = len(sim_obj.joint_handler_ids)

    sim_obj.obj_handler_ids = list(obj_handler.__dict__.values())

    sim_obj.sim.startSimulation()


def init_position():
    sim_obj.client.step()
    #Starting initial perceptions
    sim_obj.initial_j_positions = []
    #Get initial perceptions
    base_x, base_y, base_z = sim_obj.sim.getObjectPosition(obj_handler.C1a, sim_obj.sim.handle_world)
    sim_obj.initial_pos_x = base_x
    sim_obj.initial_pos_x = base_y
    sim_obj.initial_pos_x = base_z
    
    #Joint perceptions
    for joint_i_per in sim_obj.joint_handler_ids:
        #Angular/linear position of the joint
        pos = sim_obj.sim.getJointPosition(joint_i_per)
        sim_obj.initial_j_positions.append(pos)



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
        perception.prev_pos_x = sim_obj.training_data[-1]['post_pos_x']
        perception.prev_pos_y = sim_obj.training_data[-1]['post_pos_y']
        perception.prev_pos_z = sim_obj.training_data[-1]['post_pos_z']
        perception.prev_j_positions = sim_obj.training_data[-1]['post_j_positions']  
    else:
        perception.prev_pos_x = sim_obj.initial_pos_x 
        perception.prev_pos_y = sim_obj.initial_pos_y
        perception.prev_pos_z = sim_obj.initial_pos_z
        perception.prev_j_positions = sim_obj.initial_j_positions

    #Get perceptions
    base_x, base_y, base_z = sim_obj.sim.getObjectPosition(obj_handler.C1a, sim_obj.sim.handle_world)
    perception.post_pos_x = base_x
    perception.post_pos_y = base_y
    perception.post_pos_z = base_z
    perception.delta_pos_x = perception.post_pos_x - perception.prev_pos_x
    perception.delta_pos_y = perception.post_pos_y - perception.prev_pos_y
    perception.delta_pos_z = perception.post_pos_z - perception.prev_pos_z
        
    #Joint perceptions
    for joint_per in sim_obj.joint_handler_ids:
        #Angular/linear position of the joint
        pos = sim_obj.sim.getJointPosition(joint_per)
        perception.post_j_positions.append(pos)


def export():    
    #Dump data in file
    path = "C:\\Users\\carlo\\OneDrive\\Imágenes\\Documentos\\GitHub\\EMERGE\\Morphology\\Multimodular\\training_data\\"
    timestr = time.strftime("_%Y_%d%m")
    sim_obj.file = open(path + 'training_dataset_' + scene + "_" + str(port_conexion) + timestr + '.pkl', 'wb')
    pickle.dump(sim_obj.training_data, sim_obj.file)
    sim_obj.file.close()
    if not cluster:
        df = pd.DataFrame(sim_obj.training_data)
        df.to_csv(path + 'data_' + scene + "_" + str(port_conexion) + timestr + '.csv', index=False)


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