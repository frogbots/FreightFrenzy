package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

public class DuckyWheel implements MovementPerformer {


        @Override
        public void run() {
            Globals.DUCKwheel.setPower(-1);
            new SleepAction(3200).run();
            Globals.DUCKwheel.setPower(0);
        }



}
