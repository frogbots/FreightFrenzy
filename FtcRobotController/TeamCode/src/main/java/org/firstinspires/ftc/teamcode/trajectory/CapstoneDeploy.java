package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.Globals;

public class CapstoneDeploy implements MovementPerformer {


        @Override
        public void run() {

            Globals.TSELift.setPosition(.6);
            new SleepAction(1000).run();
            Globals.TSEClaw.setPosition(.4);
            new SleepAction(500).run();
            Globals.TSELift.setPosition(.95);


        }


}
