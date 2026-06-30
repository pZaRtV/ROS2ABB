MODULE Hybrid_Control
    PERS jointtarget ros_target := [[45,20,10,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS bool execute_ros_move := FALSE;
    PERS bool start_egm := FALSE;
    VAR egmident egmID1;

    PROC main()
        WHILE TRUE DO
            ! 1. RWS Control: Move to safe starting posture
            IF execute_ros_move THEN
                execute_ros_move := FALSE;
                MoveAbsJ ros_target, v100, fine, tool0;
                TPWrite "Moved to ROS target. Waiting for next command...";
            ENDIF

            ! 2. Switch to EGM Control
            IF start_egm THEN
                TPWrite "Starting EGM Stream...";
                EGMGetId egmID1;
                EGMSetupUC ROB_1, egmID1, "default", "UCdevice" \Joint;
                EGMActJoint egmID1 \Tool:=tool0 \WObj:=wobj0 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=1000;
                
                ! Run EGM for 1 hour
                EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J6 \CondTime:=3600;
                
                start_egm := FALSE;
                TPWrite "EGM finished. Back to idle.";
            ENDIF
            
            WaitTime 0.1;
        ENDWHILE
    ENDPROC
ENDMODULE
