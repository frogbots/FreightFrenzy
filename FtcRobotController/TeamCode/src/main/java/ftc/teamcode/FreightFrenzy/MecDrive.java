package ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.StateMTrajectory;

import ftc.teamcode.Toggler;

@TeleOp
public class MecDrive extends LinearOpMode {


        private DcMotorEx FL;
        private DcMotorEx FR;
        private DcMotorEx RL;
        private DcMotorEx RR;
        private double leftStickX;
        private double leftStickY;
        private double rightStickX;
        StateMTrajectory trajectory;
        SkyStoneDriveBase skyStoneDriveBase;


        AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();


        LynxDcMotorController ctrl;
        LynxModule module;

        //private Servo Tpusher;

        // private TouchSensor touch;
        // private RevTouchSensor touch;'
        // private DigitalChannel touch;



        @Override
        public void runOpMode() throws InterruptedException {


            trackingWheelIntegrator = new TrackingWheelIntegrator();

            //maping out the robot
            FL= (DcMotorEx) hardwareMap.get(DcMotor.class, "FL");
            FR= (DcMotorEx) hardwareMap.get(DcMotor.class, "FR");
            RL= (DcMotorEx) hardwareMap.get(DcMotor.class, "RL");
            RR= (DcMotorEx) hardwareMap.get(DcMotor.class, "RR");
            skyStoneDriveBase = new SkyStoneDriveBase();
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            Globals.odoModule = module;
            Globals.opMode = this;
            Globals.robot.enableBrake(true);


            boolean AutomationLastState = false;
            acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Status", "Initialized");
            telemetry.update();


            waitForStart();

            while (opModeIsActive()) {

                if (gamepad2.x) {         //If button is pressed Auto aline will run. if not normaly gamepad works
                    if (AutomationLastState == false) {
                        //clearEnc();
                        trackingWheelIntegrator.setFirstTrackingVal(0,0);
                        //buildTrajectory();

                    }
                    AutomationLastState = true;
                    trajectory.followInteration();

                }
                else {
                    runGamepad();
                    if (AutomationLastState == true) {
                        AutomationLastState = false;
                        Globals.robot.enableBrake(true);
                        Globals.robot.disablePID();

                        trajectory.reset();

                    }
                }
            }
        }

        void runGamepad() {

            if (gamepad1.left_stick_x > .01) {
                leftStickX = gamepad1.left_stick_x;
            }
            else if (gamepad1.left_stick_x < -.01) {
                leftStickX = gamepad1.left_stick_x;
            }
            else {
                leftStickX = 0;

            }
            if (gamepad1.left_stick_y > .01 ) {
                leftStickY = gamepad1.left_stick_y;
            }
            else if (gamepad1.left_stick_y < -.01) {
                leftStickY = gamepad1.left_stick_y;
            }
            else {
                leftStickY = 0;

            }
            if (gamepad1.right_stick_x > .01 ) {
                rightStickX = gamepad1.right_stick_x;
            }
            else if (gamepad1.right_stick_x < -.01) {
                rightStickX = gamepad1.right_stick_x;
            }
            else {
                rightStickX = 0;

            }
            /*if (gamepad1.left_stick_y > .03 && gamepad1.left_stick_y < -.03) {
                leftStickY = gamepad1.left_stick_y;
            }
            else {
                leftStickY = 0;
            }

             */
            /*
            telemetry.addData("Y-stick", gamepad1.left_stick_y);
            telemetry.addData("X-stick", gamepad1.left_stick_x);
            telemetry.addData("Turning", gamepad1.right_stick_x);
            telemetry.addData("Y-stick", leftStickY);
            telemetry.addData("X-stick", rightStickX);
            telemetry.update();
            */
            telemetry.addData("FL", FL.getCurrentPosition());
            telemetry.addData("FR", FR.getCurrentPosition());
            telemetry.addData("RL", RL.getCurrentPosition());
            telemetry.addData("RR", RR.getCurrentPosition());
            telemetry.update();
            MecanumDrive.cartesian(Globals.robot,
                    -leftStickY * .15, // Main
                    leftStickX * .15, // Strafe
                    rightStickX * .10); // Turn


    }
}
