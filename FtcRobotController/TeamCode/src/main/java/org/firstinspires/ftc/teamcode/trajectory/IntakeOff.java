package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

import static org.firstinspires.ftc.teamcode.Globals.RotationI;


public class IntakeOff implements MovementPerformer {



    @Override
    public void run() {
        RotationI.setPosition(.9);
        //new SleepAction(800);
        //Globals.Intake.setPower(0);


    }
}
