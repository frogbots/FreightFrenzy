package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Globals;


public class TSEDeploy implements MovementPerformer {

    public static int Lvl;

    @Override
    public void run() {

        Globals.TSELift.setPosition(1);
        /*

        Globals.Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Globals.Lift.setTargetPosition(1);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Globals.LiftLevel = 3;
        Globals.Lift.setTargetPosition(100);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.Lift.setPower(1);
        Globals.TSELift.setPosition(.7);
        new SleepAction(1000).run();
        Globals.TSEClaw.setPosition(0);
        new SleepAction(800).run();
        Globals.TSELift.setPosition(1);



         */



    }
}
