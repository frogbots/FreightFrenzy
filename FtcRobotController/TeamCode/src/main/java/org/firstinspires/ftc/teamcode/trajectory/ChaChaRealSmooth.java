package org.firstinspires.ftc.teamcode.trajectory;

import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.RotationI;


public class ChaChaRealSmooth implements MovementPerformer {

    double FAILSAFE;
    public static boolean FAILED;
    boolean fright =false;
    
    @Override
    public void run() {

        Globals.Intake.setPower(1);
        Globals.Booper.setPosition(.55);
        Globals.RotationI.setPosition(.15);
        SmoothSlide();
        if (Globals.FrightDistance > 68) {
            fright = true;
        }
        else {
            Globals.Intake.setPower(1);
            Globals.Booper.setPosition(.55);
            Globals.RotationI.setPosition(.15);
            SmoothSlide();
            if (Globals.FrightDistance > 68) {
                fright = true;
            }
            else {
                Globals.Intake.setPower(1);
                Globals.Booper.setPosition(.55);
                Globals.RotationI.setPosition(.15);
                SmoothSlide();
                if (Globals.FrightDistance > 68) {
                    fright = true;
                }
                else {
                    Globals.Intake.setPower(1);
                    Globals.Booper.setPosition(.55);
                    Globals.RotationI.setPosition(.15);
                    SmoothSlide();
                    if (Globals.FrightDistance > 68) {
                        fright = true;
                    }
                    else {
                        Globals.Intake.setPower(1);
                        Globals.Booper.setPosition(.55);
                        Globals.RotationI.setPosition(.15);
                        SmoothSlide();
                        if (Globals.FrightDistance > 68) {
                            fright = true;
                        }
                        else {

                        }
                    }
                }
            }

        }



    }
    public void SmoothSlide() {

            Globals.FL.setTargetPosition(-300);
            Globals.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Globals.FL.setPower(.9);
            Globals.RL.setTargetPosition(-300);
            Globals.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Globals.RL.setPower(.9);
            Globals.RR.setTargetPosition(-300);
            Globals.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Globals.RR.setPower(.9);
            Globals.FR.setTargetPosition(-300);
            Globals.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Globals.FR.setPower(.9);

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