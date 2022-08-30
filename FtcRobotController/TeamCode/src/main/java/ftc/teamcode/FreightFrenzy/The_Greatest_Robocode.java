package ftc.teamcode.FreightFrenzy;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.trajectory.DuckyWheel;

@TeleOp
public class The_Greatest_Robocode extends TunableLinearOpMode {

    private DcMotor WR;
    private DcMotor WL;
    private DcMotor Track;
    private DcMotorEx Lift;
    private Servo Claw;
    private double ClawPos;
    double LiftEc;
    private CRServo DuckyWheelly;

    @Override
    public void runOpMode() throws InterruptedException {

//WL = wheel left, WR = wheel right//

        WL= (DcMotor) hardwareMap.get(DcMotor.class, "WL");
        WR= (DcMotor) hardwareMap.get(DcMotor.class, "WR");
        Track= (DcMotor) hardwareMap.get(DcMotor.class, "Track");
        Lift = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "Lift");
        DuckyWheelly = (CRServo) hardwareMap.get(CRServo.class, "DuckyWheel");
        Claw = (Servo) hardwareMap.get(Servo.class, "Claw");
        WR.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()){


            ClawPos = getDouble("ClawPos");
            LiftEc = Lift.getCurrentPosition();
            telemetry.addData("LiftEc", LiftEc);
            telemetry.update();

            //WL.setPower(gamepad1.left_stick_y);  //Y=X^2
            //WR.setPower(gamepad1.right_stick_y);

            WL.setPower((gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y));
            WR.setPower((gamepad1.right_stick_y*gamepad1.right_stick_y*gamepad1.right_stick_y));

            /*if (gamepad1.right_trigger > .5) {
                Track.setPower(1);
            }

            if (gamepad1.left_trigger > .5) {
                Track.setPower(-1);
            }
            if (gamepad1.left_trigger < .5 &&  gamepad1.right_trigger < .5) {
                Track.setPower(0);
            }

             */
            /*while (gamepad1.dpad_up) {
                Lift.setPower(.5);
            }
             while (gamepad1.dpad_down) {
                Lift.setPower(-.5);
            }
            */
            
            if (gamepad1.right_bumper) {
                DuckyWheelly.setPower(1);
            }
            if (gamepad1.left_bumper) {
                DuckyWheelly.setPower(-1);
            }
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                DuckyWheelly.setPower(0);
            }
            if (gamepad1.x) {
                Claw.setPosition(.6);
            }
            if (gamepad1.y) {
                Claw.setPosition(.4);
            }
            if (gamepad1.a) {
                Lift.setTargetPosition(900);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(.5);

            }
            if (gamepad1.b) {
                Lift.setTargetPosition(0);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(.5);

            }

        }
    }
}