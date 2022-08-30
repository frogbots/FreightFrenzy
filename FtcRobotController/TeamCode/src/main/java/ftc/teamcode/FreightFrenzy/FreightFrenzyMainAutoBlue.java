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
import org.firstinspires.ftc.teamcode.trajectory.ChaChaRealSmooth;
import org.firstinspires.ftc.teamcode.trajectory.FFIntakeOn;
import org.firstinspires.ftc.teamcode.trajectory.FFPrepareForTele;
import org.firstinspires.ftc.teamcode.trajectory.IntakeOff;
import org.firstinspires.ftc.teamcode.trajectory.LiftDown;
import org.firstinspires.ftc.teamcode.trajectory.LiftPreLoad;
import org.firstinspires.ftc.teamcode.trajectory.LiftUp;
import org.firstinspires.ftc.teamcode.trajectory.PlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.PointApproach;
import org.firstinspires.ftc.teamcode.trajectory.PreLoadPlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.ResetOdo;
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

//@Autonomous(preselectTeleOp="FrogTeleOpFF")
public class FreightFrenzyMainAutoBlue extends LinearOpMode {
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
                position = DropPositions.C;; //TSEPipline.GetPosition();            THIS IS WHY VISION WONT DETECT OTHER LEVELS
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



                    .addMovement(new TSEDeploy())
                    .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-59,20)
                            .setMaxPower(1)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(1)
                            .setMovementThresh(3)
                            .setHeadingThreshold(5)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new LiftPreLoad())
                   .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-57,22)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.8)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new PreLoadPlaceMineral())
                   .addMovement(new LiftDown())//1st
                   .addMovement(new PointApproach.Builder() //Enter     //START 2
                            .setTargetPosition(-74,GapY1)
                            .setMaxPower(0.6)
                            .setXyGain(.06)
                            .setTargetHeading(GapHeading)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(3)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new FFIntakeOn())/*
                   .addMovement(new Waypoint.Builder() //Going in
                            .setTargetPosition(-95, GapY1)
                            .setTargetHeading(-269)
                            .setSpeed(.6)
                            .setTransThreshMethod(Waypoint.TranslationThreshMethod.X_ONLY)
                            .setMovementThresh(5)
                            .setHeadingThreshold(0)
                           .build())
                   .addMovement(new PointApproach.Builder() //Pickup
                            .setTargetPosition(-106,3)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-270)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new IntakeOff())
                   .addMovement(new PointApproach.Builder() //Pickup
                            .setTargetPosition(-100,3)
                            .setMaxPower(0.7)
                            .setXyGain(.06)
                            .setTargetHeading(-270)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                   .addMovement(new Transfer())
                   .addMovement(new TouchWALL())
                    .addMovement(new SleepAction(1000))
                   .addMovement(new Waypoint.Builder() // First Stright movment to Target position
                            .setTargetPosition(-80, GapY1)
                            .setTargetHeading(-271)
                            .setSpeed(.6)
                            .setTransThreshMethod(Waypoint.TranslationThreshMethod.X_ONLY)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(0)
                            .build())

                            */
                   .addMovement(new PointApproach.Builder() //Move into
                            .setTargetPosition(-100,GapY1)
                            .setMaxPower(.6)
                            .setXyGain(.06)
                            .setTargetHeading(GapHeading+2)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                   .addMovement(new PointApproach.Builder() //Move into
                            .setTargetPosition(-106,GapY1)
                            .setMaxPower(0.25)
                            .setXyGain(.06)
                            .setTargetHeading(GapHeading+2)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new IntakeOff())
                   .addMovement(new PointApproach.Builder() //Going out
                            .setTargetPosition(-100,GapY1)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(GapHeading+2)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                   .addMovement(new Transfer())
                   .addMovement(new PointApproach.Builder() //Out
                            .setTargetPosition(-74,GapY1)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-270)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                   .addMovement(new LiftUp())
                    /*
                   .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-59,22)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.8)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())

                            */
                   .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-57,18)
                            .setMaxPower(1)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(1)
                            .setMovementThresh(3)
                            .setHeadingThreshold(5)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-55,20)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.8)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                   .addMovement(new PlaceMineral())
                   .addMovement(new LiftDown()) //2nd
                   .addMovement(new PointApproach.Builder() //Enter                    //START 3
                            .setTargetPosition(-74,GapY2)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(GapHeading+4)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                   // .addMovement(new LiftDown())
                   .addMovement(new FFIntakeOn())
                   .addMovement(new PointApproach.Builder() //IN
                            .setTargetPosition(-100,GapY2)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                   .addMovement(new PointApproach.Builder() //Move into
                            .setTargetPosition(-108.5,GapY2)
                            .setMaxPower(0.25)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(true)
                            .build())
                    //.addMovement(new ChaChaRealSmooth())
                   .addMovement(new IntakeOff())
                   .addMovement(new PointApproach.Builder() //Going out
                            .setTargetPosition(-100, GapY2)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new Transfer())
                    .addMovement(new PointApproach.Builder() //Out
                            .setTargetPosition(-74,GapY2)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.7)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new LiftUp())/*
                    .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-59,22)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.8)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                            */
                    .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-57,18)
                            .setMaxPower(1)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(1)
                            .setMovementThresh(3)
                            .setHeadingThreshold(5)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new PointApproach.Builder() //Shipping Hub movment
                            .setTargetPosition(-57,20)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.8)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new PlaceMineral())
                    .addMovement(new LiftDown()) //3rd

                    .addMovement(new PointApproach.Builder() //Enter
                            .setTargetPosition(-74,GapY3)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(true)
                            .build())
                    //.addMovement(new LiftDown())
                    .addMovement(new FFIntakeOn())
                    .addMovement(new PointApproach.Builder() //IN
                            .setTargetPosition(-100,GapY3)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.5)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new PointApproach.Builder() //Move into
                            .setTargetPosition(-110,GapY3)
                            .setMaxPower(0.1)
                            .setXyGain(.06)
                            .setTargetHeading(-266)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(1)
                            .stopMotorsOnDone(true)
                            .build())
                    //.addMovement(new ChaChaRealSmooth())
                    .addMovement(new IntakeOff())


                    .addMovement(new FFPrepareForTele())
                    .addMovement(new LiftDown())
                    .addMovement(new SleepAction(500))
                    .addMovement(new Transfer())


                    .addMovement(new SleepAction(2000))

                    .build();
        }


}



  /*
                    .addMovement(new FFIntakeOn())
                    .addMovement(new PointApproach.Builder() //Movement In Warehouse
                            .setTargetPosition(106,-1)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(265)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new FreightDetection())
                    .addMovement(new PointApproach.Builder() //Move to Leave
                            .setTargetPosition(87,-1)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(265)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new PointApproach.Builder() //move to hub
                            .setTargetPosition(60,10)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(200)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    //.addMovement(new BounceBackPSOneWithNoWobbleGoal())

                     */