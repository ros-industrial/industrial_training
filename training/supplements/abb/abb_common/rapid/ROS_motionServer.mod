MODULE ROS_motionServer

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

LOCAL CONST num server_port := 11000;

LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;
LOCAL VAR ROS_joint_trajectory_pt trajectory{MAX_TRAJ_LENGTH};
LOCAL VAR num trajectory_size;

PROC main()
    VAR ROS_msg_joint_traj_pt message;

    TPWrite "MotionServer: Waiting for connection.";
	ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket;

    WHILE ( true ) DO
		! Recieve Joint Trajectory Pt Message
        ROS_receive_msg_joint_traj_pt client_socket, message;
		trajectory_pt_callback message;
	ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED)
	IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "ROS MotionServer disconnect", "Connection lost.  Resetting socket.";
		ExitCycle;  ! restart program
	ELSE
		TRYNEXT;
	ENDIF
UNDO
	IF (SocketGetStatus(client_socket) <> SOCKET_CLOSED) SocketClose client_socket;
	IF (SocketGetStatus(server_socket) <> SOCKET_CLOSED) SocketClose server_socket;
ENDPROC

LOCAL PROC trajectory_pt_callback(ROS_msg_joint_traj_pt message)
	VAR ROS_joint_trajectory_pt point;
	VAR jointtarget current_pos;
    VAR ROS_msg reply_msg;

    point := [message.joints, message.duration];
    
    ! use sequence_id to signal start/end of trajectory download
	TEST message.sequence_id
		CASE ROS_TRAJECTORY_START_DOWNLOAD:
            TPWrite "Traj START received";
			trajectory_size := 0;                 ! Reset trajectory size
            add_traj_pt point;                    ! Add this point to the trajectory
		CASE ROS_TRAJECTORY_END:
            TPWrite "Traj END received";
            add_traj_pt point;                    ! Add this point to the trajectory
            activate_trajectory;
		CASE ROS_TRAJECTORY_STOP:
            TPWrite "Traj STOP received";
            trajectory_size := 0;  ! empty trajectory
            activate_trajectory;
            StopMove; ClearPath; StartMove;  ! redundant, but re-issue stop command just to be safe
		DEFAULT:
            add_traj_pt point;                    ! Add this point to the trajectory
	ENDTEST

    ! send reply, if requested
    IF (message.header.comm_type = ROS_COM_TYPE_SRV_REPLY) THEN
        reply_msg.header := [ROS_MSG_TYPE_JOINT_TRAJ_PT, ROS_COM_TYPE_SRV_REPLY, ROS_REPLY_TYPE_SUCCESS];
        ROS_send_msg client_socket, reply_msg;
    ENDIF

ERROR
    RAISE;  ! raise errors to calling code
ENDPROC

LOCAL PROC add_traj_pt(ROS_joint_trajectory_pt point)
    IF (trajectory_size = MAX_TRAJ_LENGTH) THEN
        ErrWrite \W, "Too Many Trajectory Points", "Trajectory has already reached its maximum size",
            \RL2:="max_size = " + ValToStr(MAX_TRAJ_LENGTH);
    ELSE
        Incr trajectory_size;
        trajectory{trajectory_size} := point; !Add this point to the trajectory
    ENDIF
ENDPROC

LOCAL PROC activate_trajectory()
    WaitTestAndSet ROS_trajectory_lock;  ! acquire data-lock
    TPWrite "Sending " + ValToStr(trajectory_size) + " points to MOTION task";
    ROS_trajectory := trajectory;
    ROS_trajectory_size := trajectory_size;
    ROS_new_trajectory := TRUE;
    ROS_trajectory_lock := FALSE;  ! release data-lock
ENDPROC
	
ENDMODULE