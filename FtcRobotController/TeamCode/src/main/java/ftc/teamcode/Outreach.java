package ftc.teamcode;


// import

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// import com.qualcomm.hardware.rev.RevTouchSensor;
// where is it

//

//build it store
@TeleOp
public class Outreach extends LinearOpMode {

    private DcMotor RightDrive;
    private DcMotor LeftDrive;
    private DcMotor Lift;
    private DcMotor RingArm;
    private Servo WGClaw;
    private Servo RingGrab;
    double ring;


    @Override
    public void runOpMode() throws InterruptedException {


        // trackingWheelIntegrator = new TrackingWheelIntegrator();

        //maping out the robot
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        RingArm = hardwareMap.get(DcMotor.class, "Lift");
        Lift = hardwareMap.get(DcMotor.class, "RingArm");
        WGClaw = hardwareMap.get(Servo.class, "WGClaw");
        RingGrab = hardwareMap.get(Servo.class, "RingGrab");


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            RightDrive.setPower(gamepad1.right_stick_y);
            LeftDrive.setPower(-gamepad1.left_stick_y);

            if (gamepad1.a) { // Wobble
                WGClaw.setPosition(1);
            }
            if (gamepad1.b) {
                WGClaw.setPosition(.85);
            }

            if (gamepad1.x) { //close position
                RingGrab.setPosition(.7);
            }
            if (gamepad1.y) {
                RingGrab.setPosition(1);
            }

            if (gamepad1.dpad_up) {
                Lift.setPower(.5);
            }
            else if (gamepad1.dpad_down) {
                Lift.setPower(-.5);
            }
            else {
                Lift.setPower(0);
            }

            if (gamepad1.dpad_left) {
                RingArm.setPower(.7);
            }
            else if (gamepad1.dpad_right) {
                RingArm.setPower(-.7);
            }
            else {
                RingArm.setPower(0);
            }
            if (gamepad1.dpad_left) {
                RingArm.setPower(1);
            }
            if (gamepad1.dpad_right) {
                RingArm.setPower(0);
            }
        }
    }
}