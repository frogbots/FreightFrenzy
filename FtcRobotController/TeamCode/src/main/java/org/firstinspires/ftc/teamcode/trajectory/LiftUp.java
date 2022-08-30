package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.Booper;
import static org.firstinspires.ftc.teamcode.Globals.Lift;


public class LiftUp implements MovementPerformer {


    @Override
    public void run() {

        Globals.Intake.setPower(0);
        Globals.TSELift.setPosition(1);
        Globals.Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Globals.Lift.setTargetPosition(1);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Globals.LiftLevel = 3;
        Globals.Lift.setTargetPosition(1000);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.Lift.setPower(1);



    }
}
