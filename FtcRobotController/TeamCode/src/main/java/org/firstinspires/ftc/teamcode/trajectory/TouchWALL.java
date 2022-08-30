package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;


public class TouchWALL implements MovementPerformer {

    double FAILSAFE;
    public static boolean FAILED;
    boolean fright =false;
    
    @Override
    public void run() {

        Globals.FL.setTargetPosition(300);
        Globals.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.RL.setTargetPosition(-300);
        Globals.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.RR.setTargetPosition(300);
        Globals.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.FR.setTargetPosition(-300);
        Globals.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.RR.setPower(.9);
        Globals.FR.setPower(.9);
        Globals.FL.setPower(.9);
        Globals.RL.setPower(.9);
        //new SleepAction(1000).run();
    }

}