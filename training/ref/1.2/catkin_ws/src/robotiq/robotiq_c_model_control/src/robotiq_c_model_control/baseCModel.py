# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Module baseCModel: defines a base class for handling command and status of the Robotiq C-Model gripper. 

After being instanciated, a 'client' member must be added to the object. This client depends on the communication protocol used by the Gripper. As an example, the ROS node 'CModelTcpNode.py' instanciate a robotiqBaseCModel and adds a client defined in the module comModbusTcp.
"""

from   robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
from   robotiq_c_model_control.msg import _CModel_robot_output as outputMsg

class robotiqBaseCModel:
    """Base class (communication protocol agnostic) for sending commands and receiving the status of the Robotic C-Model gripper"""

    def __init__(self):

        #Initiate output message as an empty list
        self.message = []

        #Note: after the instantiation, a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Function to verify that the value of each variable satisfy its limits."""
    	   	
   	#Verify that each variable is in its correct range
   	command.rACT = max(0, command.rACT)
   	command.rACT = min(1, command.rACT)
   	
   	command.rGTO = max(0, command.rGTO)
   	command.rGTO = min(1, command.rGTO)

   	command.rATR = max(0, command.rATR)
   	command.rATR = min(1, command.rATR)
   	
   	command.rPR  = max(0,   command.rPR)
   	command.rPR  = min(255, command.rPR)   	

   	command.rSP  = max(0,   command.rSP)
   	command.rSP  = min(255, command.rSP)   	

   	command.rFR  = max(0,   command.rFR)
   	command.rFR  = min(255, command.rFR) 
   	
   	#Return the modified command
   	return command

    def refreshCommand(self, command):
        """Function to update the command which will be sent during the next sendCommand() call."""
    
	#Limit the value of each variable
    	command = self.verifyCommand(command)

        #Initiate command as an empty list
        self.message = []

        #Build the command with each output variable
        #To-Do: add verification that all variables are in their authorized range
        self.message.append(command.rACT + (command.rGTO << 3) + (command.rATR << 4))
        self.message.append(0)
        self.message.append(0)
        self.message.append(command.rPR)
        self.message.append(command.rSP)
        self.message.append(command.rFR)     

    def sendCommand(self):
        """Send the command to the Gripper."""    
        
        self.client.sendCommand(self.message)

    def getStatus(self):
        """Request the status from the gripper and return it in the CModel_robot_input msg type."""    

        #Acquire status from the Gripper
        status = self.client.getStatus(6);

        #Message to output
        message = inputMsg.CModel_robot_input()

        #Assign the values to their respective variables
        message.gACT = (status[0] >> 0) & 0x01;        
        message.gGTO = (status[0] >> 3) & 0x01;
        message.gSTA = (status[0] >> 4) & 0x03;
        message.gOBJ = (status[0] >> 6) & 0x03;
        message.gFLT =  status[2]
        message.gPR  =  status[3]
        message.gPO  =  status[4]
        message.gCU  =  status[5]       

        return message
        
