package ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.AutoTransfer;
import org.firstinspires.ftc.teamcode.trajectory.SMtransfer;
import org.firstinspires.ftc.teamcode.trajectory.StateMMovmentPerformer;
import org.firstinspires.ftc.teamcode.trajectory.StateMTrajectory;

import ftc.teamcode.Toggler;

@TeleOp
public class OdoWheelsReset extends LinearOpMode {


        private DcMotorEx FL;
        private DcMotorEx FR;
        private DcMotorEx RL;
        private DcMotorEx RR;
        private DcMotor rightTW;
        private DcMotor Lift;
        private DcMotor leftTW;
        private DcMotor backTW;
        private DcMotor Intake;
        private Servo Dumper;
        private Servo RotationI;
        private Servo TSELift;
        private Servo TSEClaw;
        //private Servo DUCKwheel;
        private Servo Booper;
        Toggler LiftUp = new Toggler();
        Toggler LiftDown = new Toggler();
        StateMTrajectory trajectory;
        SkyStoneDriveBase skyStoneDriveBase;
        private CRServo DUCKwheel;
        private CRServo RTW;
        private CRServo LTW;
        private CRServo ATW;
        double LeftPos;
        double RotationIntake;
        int LiftAdjustment;
        boolean FrightDetection;
        long elapsedTime;
        int LiftPosition=0;
        int LiftHeight;
        TouchSensor LiftLimit;





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
            Globals.LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit");
            TSELift = hardwareMap.get(Servo.class, "TSELift");
            TSEClaw = hardwareMap.get(Servo.class, "TSEClaw");
            Booper = hardwareMap.get(Servo.class, "Dumper");
            Globals.RotationI = hardwareMap.get(Servo.class, "RotationI");
            DUCKwheel = hardwareMap.get(CRServo.class, "DUCKwheel");
            RTW = hardwareMap.get(CRServo.class, "RTW");
            LTW = hardwareMap.get(CRServo.class, "LTW");
            ATW = hardwareMap.get(CRServo.class, "ATW");
            Globals.Lift = hardwareMap.get(DcMotorEx.class, "Lift");
            leftTW = hardwareMap.get(DcMotor.class, "Intake"); // this is also left Tracking wheel
            rightTW = hardwareMap.get(DcMotor.class, "brokport");
            Globals.Intake = hardwareMap.get(DcMotor.class, "Intake");
            backTW = hardwareMap.get(DcMotor.class, "trakingport");
            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
            Globals.FrightDetector = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "REVDS");
            // imu = new FrogBNO055( hardwareMap.get(BNO055IMU.class, "external_IMU"));
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            //Globals.Backsonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
            //Globals.LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
            //Globals.RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
            Globals.odoModule = module;
            Globals.opMode = this;
            Globals.robot.enableBrake(true);


            Globals.Lift.setDirection(DcMotorSimple.Direction.REVERSE);
            Globals.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boolean AutomationLastState = false;
            acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Status", "Initialized");
            telemetry.update();


            //clearEnc();
            //imu.init();

            buildTrajectory();
            waitForStart();

            while (opModeIsActive()) {

                Globals.updateTracking();
                if (gamepad1.right_bumper) {         //If button is pressed Auto aline will run. if not normaly gamepad works
                    if (AutomationLastState == false) {
                        //clearEnc();
                        //trackingWheelIntegrator.setFirstTrackingVal(0,0);
                        transferSM();

                    }
                    runGamepad();
                    AutomationLastState = true;
                    trajectory.followInteration();

                }
                /*else if (FrightDistance < 68 && FrightDetection == false) {
                    gamepad1.rumbleBlips(2);
                    if (AutomationLastState == false) {
                        buildTrajectory();

                    }
                    runGamepad();
                    AutomationLastState = true;
                    trajectory.followInteration();

                }

                 */
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
            while (gamepad1.a) {
                RTW.setPower(gamepad1.left_stick_x);
            }
            while (gamepad1.b) {
                LTW.setPower(gamepad1.left_stick_x);
            }
            while (gamepad1.x) {
                ATW.setPower(gamepad1.left_stick_x);
            }


    }
    public void transferSM() {
        trajectory = new StateMTrajectory.Builder()
                .addMovement((StateMMovmentPerformer) new SMtransfer())

                .build();
    }
    public void buildTrajectory() {

        trajectory = new StateMTrajectory.Builder()
                .addMovement(new AutoTransfer())
                .build();
    }
}
