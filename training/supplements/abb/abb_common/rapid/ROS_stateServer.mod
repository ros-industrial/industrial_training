MODULE ROS_stateServer

! Software License Agreement (BSD License)
!
! Copyright (c) 2012, Edward Venator, Case Western Reserve University
! Copyright (c) 2012, Jeremy Zoss, Southwest Research Institute
! All rights reserved.
!
! Redistribution and use in source and binary forms, with or without modification,
! are permitted provided that the following conditions are met:
!
!   Redistributions of source code must retain the above copyright notice, this
!       list of conditions and the following disclaimer.
!   Redistributions in binary form must reproduce the above copyright notice, this
!       list of conditions and the following disclaimer in the documentation
!       and/or other materials provided with the distribution.
!   Neither the name of the Case Western Reserve University nor the names of its contributors
!       may be used to endorse or promote products derived from this software without
!       specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
! EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
! OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
! SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
! TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
! BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
! WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

LOCAL CONST num server_port := 11002;
LOCAL CONST num update_rate := 0.10;  ! broadcast rate (sec)

LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;

PROC main()

    TPWrite "StateServer: Waiting for connection.";
	ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket;
    
	WHILE (TRUE) DO
		send_joints;
		WaitTime update_rate;
    ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED)
	IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "ROS StateServer disconnect", "Connection lost.  Waiting for new connection.";
        ExitCycle;  ! restart program
	ELSE
		TRYNEXT;
	ENDIF
UNDO
ENDPROC

LOCAL PROC send_joints()
	VAR ROS_msg_joint_data message;
	VAR jointtarget joints;
	
    ! get current joint position (degrees)
	joints := CJointT();
    
    ! create message
    message.header := [ROS_MSG_TYPE_JOINT, ROS_COM_TYPE_TOPIC, ROS_REPLY_TYPE_INVALID];
    message.sequence_id := 0;
    message.joints := joints.robax;
    
    ! send message to client
    ROS_send_msg_joint_data client_socket, message;

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

ENDMODULE