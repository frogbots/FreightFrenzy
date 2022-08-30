package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.Booper;
import static org.firstinspires.ftc.teamcode.Globals.Lift;
import static org.firstinspires.ftc.teamcode.Globals.TSELift;


public class PlaceMineral implements MovementPerformer {


    @Override
    public void run() {





        TSELift.setPosition(.9);
        Booper.setPosition(.078);
        new SleepAction(700).run();
        Booper.setPosition(.5);


    }
}
/*
Globals.Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Globals.Lift.setTargetPosition(1);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Globals.LiftLevel = 3;
        Globals.Lift.setTargetPosition(830);
        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Globals.Lift.setPower(1);
        while (Lift.getCurrentPosition()>830) {
        }
        new SleepAction(600).run();
 */