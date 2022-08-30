package org.firstinspires.ftc.teamcode.NotFF;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.drivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.StateMTrajectory;

//@TeleOp
public class FrogTeleOp extends LinearOpMode {


        private DcMotorEx FL;
        private DcMotorEx FR;
        private DcMotorEx RL;
        private DcMotorEx RR;
        private DcMotor rightTW;
        private DcMotor leftTW;
        private DcMotor backTW;
        StateMTrajectory trajectory;
        SkyStoneDriveBase skyStoneDriveBase;




        AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();


        LynxDcMotorController ctrl;
        LynxModule module;

    /*public static float getYaw() {

    }

     */


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
            leftTW = hardwareMap.get(DcMotor.class, "Intake"); // this is also left Tracking wheel
            rightTW = hardwareMap.get(DcMotor.class, "brokport");
            backTW = hardwareMap.get(DcMotor.class, "shooter");
            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
            // imu = new FrogBNO055( hardwareMap.get(BNO055IMU.class, "external_IMU"));
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            Globals.Backsonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
            Globals.LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
            Globals.RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
            Globals.odoModule = module;
            Globals.opMode = this;
            Globals.robot.enableBrake(true);

            boolean AutomationLastState = false;
            acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //buildTrajectory();
            waitForStart();

            while (opModeIsActive()) {

                if (gamepad2.left_bumper) {         //If button is pressed Auto aline will run. if not normaly gamepad works
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

            MecanumDrive.cartesian(Globals.robot,
                    -gamepad1.left_stick_y, // Main
                    gamepad1.left_stick_x , // Strafe
                    gamepad1.right_stick_x * .85); // Turn


            if (gamepad1.a) {} //Intake
            if (gamepad1.b) {}
            if (gamepad1.y) {}
            if (gamepad1.dpad_up) {}
            if (gamepad1.dpad_down) {}
            if (gamepad2.b) {}

    }






}
