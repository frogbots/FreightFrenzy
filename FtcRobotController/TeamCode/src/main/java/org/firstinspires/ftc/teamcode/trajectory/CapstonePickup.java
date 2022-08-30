package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

public class CapstonePickup implements MovementPerformer {


        @Override
        public void run() {

            Globals.TSEClaw.setPosition(.2);
            new SleepAction(1000).run();


        }


}
