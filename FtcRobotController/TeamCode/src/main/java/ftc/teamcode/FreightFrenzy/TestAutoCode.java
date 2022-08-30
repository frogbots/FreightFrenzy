package ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.control.AcceleratedGain;
import org.firstinspires.ftc.teamcode.drivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.FFIntakeOn;
import org.firstinspires.ftc.teamcode.trajectory.FFPrepareForTele;
import org.firstinspires.ftc.teamcode.trajectory.IntakeOff;
import org.firstinspires.ftc.teamcode.trajectory.LiftDown;
import org.firstinspires.ftc.teamcode.trajectory.LiftPreLoad;
import org.firstinspires.ftc.teamcode.trajectory.LiftUp;
import org.firstinspires.ftc.teamcode.trajectory.PlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.PointApproach;
import org.firstinspires.ftc.teamcode.trajectory.PreLoadPlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.SleepAction;
import org.firstinspires.ftc.teamcode.trajectory.TSEDeploy;
import org.firstinspires.ftc.teamcode.trajectory.TouchWALL;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.Transfer;
import org.firstinspires.ftc.teamcode.trajectory.Waypoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import ftc.teamcode.DropPositions;

@Autonomous(preselectTeleOp="FrogTeleOpFF")
public class TestAutoCode extends LinearOpMode {
        MovingStatistics movingStatistics = new MovingStatistics(300);

        long startLoop = 0;

        double CapstoneHeading;
        double CapstoneYPos;
        double CapstoneXPos;
        double CapstoneHeading2;
        double CapstoneYPos2;
        double CapstoneXPos2;
        double GapHeading;
        double GapY1;
        double GapY2;
        double GapY3;

        double cmToInch = 0;
        public double inch = 0;
        //OpenCvCamera phoneCam;
        public static boolean RingStack;
        OpenCvCamera phoneCam;


        private DcMotor Intake;
        private Servo Dumper;
        private Servo RotationI;
        private Servo TSELift;
        private Servo TSEClaw;
        //private Servo DUCKwheel;
        private Servo Booper;


        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

        static LynxDcMotorController ctrl;
        LynxModule module;

        SkyStoneDriveBase skyStoneDriveBase;

        Trajectory trajectory;

        DropPositions position = DropPositions.B;

        @Override
        public void runOpMode() throws InterruptedException
        {

            trackingWheelIntegrator = new TrackingWheelIntegrator();

            Globals.FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
            Globals.TSELift = hardwareMap.get(Servo.class, "TSELift");
            Globals.TSEClaw = hardwareMap.get(Servo.class, "TSEClaw");
            Globals.Booper = hardwareMap.get(Servo.class, "Dumper");
            Globals.RotationI = hardwareMap.get(Servo.class, "RotationI");
            Globals.FL = hardwareMap.get(DcMotorEx.class, "FL");
            Globals.FR = hardwareMap.get(DcMotorEx.class, "FR");
            Globals.RR = hardwareMap.get(DcMotorEx.class, "RR");
            Globals.RL = hardwareMap.get(DcMotorEx.class, "RL");
            Globals.DUCKwheel = hardwareMap.get(CRServo.class, "DUCKwheel");
            Globals.Lift = hardwareMap.get(DcMotorEx.class, "Lift");
            Globals.Intake = hardwareMap.get(DcMotor.class, "Intake");
            Globals.FrightDetector = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "REVDS");
            Globals.leftTW = hardwareMap.get(DcMotorEx.class, "Intake"); // this is also left Tracking wheel
            Globals.rightTW = hardwareMap.get(DcMotorEx.class, "brokport");
            Globals.backTW = hardwareMap.get(DcMotorEx.class, "trakingport");

            skyStoneDriveBase = new SkyStoneDriveBase();
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;

            Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Globals.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Globals.Lift.setTargetPosition(1);
            Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //Globals.RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
            //Globals.LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
            Globals.opMode = this;
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            Globals.odoModule = module;

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
            TSEDetectionBlue TSEPipline = new TSEDetectionBlue();
            phoneCam.setPipeline(TSEPipline);
            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            //Globals.RingDetector = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "REVCS");

            telemetry.setMsTransmissionInterval(20);
            //cmToInch = Globals.LeftSonar.getDistanceSync();
            //inch = cmToInch/2.54;
            //telemetry.addData("Dist inch", -inch+2);
            telemetry.addData("Position", TSEPipline.GetPosition());
            telemetry.update();

            Globals.inch = inch;

            trackingWheelIntegrator.setFirstTrackingVal(-70,0);

            clearEnc();

            while (!isStopRequested() && !isStarted()) {
                position = TSEPipline.GetPosition();
                telemetry.addData("Position", TSEPipline.GetPosition());
                //telemetry.addData("Dist inch", inch+1);
                telemetry.update();
            }
            if (position == DropPositions.A) { //Lvl 1
                CapstoneXPos= 71.6;
                CapstoneYPos= 15.5;
                CapstoneHeading= 180;
                CapstoneXPos2= 71.6;
                CapstoneYPos2= 17.7;
                CapstoneHeading2= 180;
                LiftPreLoad.Lvl = 100;
            }
            if (position == DropPositions.B) { // Lvl 2
                CapstoneXPos= 72.4;
                CapstoneYPos= 14;
                CapstoneHeading= 150;
                CapstoneXPos2= 73.8;
                CapstoneYPos2= 16.8;
                CapstoneHeading2= 146;
                LiftPreLoad.Lvl = 520;
            }
            if (position == DropPositions.C) { //Lvl 3
                CapstoneXPos= 73;
                CapstoneYPos= 19;
                CapstoneHeading= 110;
                CapstoneXPos2= 75.5;
                CapstoneYPos2= 21.7;
                CapstoneHeading2= 107;
                LiftPreLoad.Lvl = 830;
            }


            GapHeading = -270;
            GapY1 = -.5;
            GapY2 = -.5;
            GapY3 = -.5;              ;

            buildTrajectory();
            trajectory.follow();
        }
        public static void clearEnc()        {
            ctrl.setMotorMode(3, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            ctrl.setMotorMode(3, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        public void buildTrajectory()
        {
            trajectory = new Trajectory.Builder()

                    .addMovement(new TouchWALL())
                    .addMovement(new SleepAction(10000))
                    .build();
        }


}

