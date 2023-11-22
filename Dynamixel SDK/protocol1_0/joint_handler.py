#     |\__/,|   (`\
#   _.|o o  |_   ) )
# -(((---(((--------

from emerge_joint_handler import *
from sim_joint_handler import *

def JointHandler(is_sim, portH_client, packetH_sim): #is_sim, portH_client, packetH_sim
    if is_sim:
        client = portH_client
        sim = packetH_sim
        return JointHandlerSim(client, sim)
    else:
        portHandler = portH_client
        packetHandler = packetH_sim
        return JointHandlerEMERGE(portHandler, packetHandler)