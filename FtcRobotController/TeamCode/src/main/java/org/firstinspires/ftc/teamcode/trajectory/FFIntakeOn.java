package org.firstinspires.ftc.teamcode.trajectory;

import static org.firstinspires.ftc.teamcode.Globals.Intake;
import static org.firstinspires.ftc.teamcode.Globals.RotationI;

public class FFIntakeOn implements MovementPerformer {


        @Override
        public void run() {

            //new SleepAction(100).run();
            Intake.setPower(1);
            RotationI.setPosition(.15);
        }


}
