package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

@TeleOp
public class ServoTunerTest extends TunableLinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {

        Servo hexCLAW = hardwareMap.servo.get("HEXCLAW");
        Servo TSERotation = hardwareMap.servo.get("TSERotation");
        Servo TSELift = hardwareMap.servo.get("TSELift");

        waitForStart();

        while (opModeIsActive())
        {
            hexCLAW.setPosition(getDouble("servoTest"));
            TSELift.setPosition(getDouble("TSELift"));
            TSERotation.setPosition(getDouble("TSERotation"));


        }

    }
}
