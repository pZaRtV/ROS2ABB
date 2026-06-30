MODULE EGM_ROS2

    VAR egmident egmID1;

    PROC main()
        TPWrite "EGM: Step 1 - Calling EGMGetId...";
        EGMGetId egmID1;
        TPWrite "EGM: Step 2 - EGMGetId OK, calling EGMSetupUC...";

        ! Bind to UCdevice (UDPUC -> 192.168.255.128:6511)
        ! "default"   = External Motion Interface config (Motion > Configuration)
        ! "UCdevice"  = Transmission Protocol entry (Communication > Configuration)
        EGMSetupUC ROB_1, egmID1, "default", "UCdevice" \Joint;
        TPWrite "EGM: Step 3 - EGMSetupUC OK, calling EGMActJoint...";

        EGMActJoint egmID1
            \Tool:=tool0
            \WObj:=wobj0
            \MaxPosDeviation:=1000
            \MaxSpeedDeviation:=1000;
        TPWrite "EGM: Step 4 - EGMActJoint OK, calling EGMRunJoint...";

        ! Loop forever ?" EGMRunJoint restarts automatically each cycle
        ! CondTime:=3600 = 1 hour per cycle (effectively continuous)
        WHILE TRUE DO
            EGMRunJoint egmID1, EGM_STOP_HOLD
                \J1 \J2 \J3 \J6
                \CondTime:=3600;
            TPWrite "EGM: Step 5 - EGMRunJoint finished.";
        ENDWHILE

        EGMStop egmID1, EGM_STOP_HOLD;
        EGMReset egmID1;
        TPWrite "EGM: Done.";

    ERROR
        TPWrite "EGM: ERROR at ERRNO="\Num:=ERRNO;
        EGMReset egmID1;
        RAISE;
    ENDPROC

ENDMODULE