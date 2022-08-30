package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

public class DuckyWheelBlue implements MovementPerformer {


        @Override
        public void run() {
            Globals.DUCKwheel.setPower(.83);
            new SleepAction(3200).run();
            Globals.DUCKwheel.setPower(0);
        }



}
