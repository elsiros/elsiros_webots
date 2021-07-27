#import sim

class Visualisation:
    def __init__(self):
        self.pf_dummy_handle = []
        self.obstacle_handle = []

    def localisation_points(self, motion, Dummy_PF_position, Dummy_PF_orientation, Dummy_PF_H):
        pass
        #for handle in self.pf_dummy_handle:
        #    returnCode = motion.sim.simxRemoveObject(motion.clientID, handle, motion.sim.simx_opmode_oneshot )
        #self.pf_dummy_handle.clear()
        #dummy_pf_color = bytearray([0,127,0,0,0,0,64,64,64,0,0,0])
        #returnCode, pf_dummy_handle = motion.sim.simxCreateDummy(motion.clientID, 0.05, dummy_pf_color,
        #                                motion.sim.simx_opmode_blocking  )
        #self.pf_dummy_handle.append(pf_dummy_handle)
        #returnCode = motion.sim.simxSetObjectPosition(motion.clientID, pf_dummy_handle , 
        #                                                    -1,Dummy_PF_position,                            
        #                                                    motion.sim.simx_opmode_oneshot)
        #returnCode = motion.sim.simxSetObjectOrientation(motion.clientID, pf_dummy_handle , 
        #                                                    -1,Dummy_PF_orientation, motion.sim.simx_opmode_oneshot)
        #for Dummy_PF_H_position in Dummy_PF_H:
        #    returnCode, pf_dummy_handle = motion.sim.simxCreateDummy(
        #                                    motion.clientID, 0.03, dummy_pf_color,
        #                                    motion.sim.simx_opmode_blocking  ) 
        #    self.pf_dummy_handle.append(pf_dummy_handle)
        #    returnCode = motion.sim.simxSetObjectPosition(motion.clientID, pf_dummy_handle , 
        #                                                    -1,Dummy_PF_H_position,                              
        #                                                    motion.sim.simx_opmode_oneshot)

    def obstacle_mark(self, motion, obstacles):
        pass
        #for handle in self.obstacle_handle:
        #    returnCode = motion.sim.simxRemoveObject(motion.clientID, handle, motion.sim.simx_opmode_oneshot )
        #self.obstacle_handle.clear()
        #dummy_pf_color = bytearray([0,127,0,0,0,0,64,64,64,0,0,0])
        #for obstacle in obstacles:
        #    returnCode, obstacle_handle = motion.sim.simxCreateDummy(
        #                                    motion.clientID, obstacle[2], dummy_pf_color,
        #                                    motion.sim.simx_opmode_blocking  ) 
        #    self.obstacle_handle.append(obstacle_handle)
        #    returnCode = motion.sim.simxSetObjectPosition(motion.clientID, obstacle_handle , 
        #                                                    -1,[obstacle[0],obstacle[1], 0],                              
        #                                                    motion.sim.simx_opmode_oneshot)




