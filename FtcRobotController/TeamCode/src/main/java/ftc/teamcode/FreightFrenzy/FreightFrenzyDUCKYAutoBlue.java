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
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.DuckyWheel;
import org.firstinspires.ftc.teamcode.trajectory.DuckyWheelBlue;
import org.firstinspires.ftc.teamcode.trajectory.FFIntakeOn;
import org.firstinspires.ftc.teamcode.trajectory.FFPrepareForTele;
import org.firstinspires.ftc.teamcode.trajectory.IntakeOff;
import org.firstinspires.ftc.teamcode.trajectory.LiftDown;
import org.firstinspires.ftc.teamcode.trajectory.LiftPreLoad;
import org.firstinspires.ftc.teamcode.trajectory.LiftUp;
import org.firstinspires.ftc.teamcode.trajectory.PlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.PlacePreLoad;
import org.firstinspires.ftc.teamcode.trajectory.PointApproach;
import org.firstinspires.ftc.teamcode.trajectory.PreLoadPlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.SleepAction;
import org.firstinspires.ftc.teamcode.trajectory.TSEDeploy;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.Transfer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import ftc.teamcode.DropPositions;

//@Autonomous(preselectTeleOp="FrogTeleOpFF")
public class FreightFrenzyDUCKYAutoBlue extends LinearOpMode {
        MovingStatistics movingStatistics = new MovingStatistics(300);

        long startLoop = 0;



        double CapstoneHeading;
        double CapstoneYPos;
        double CapstoneXPos;
        double CapstoneHeading2;
        double CapstoneYPos2;
        double CapstoneXPos2;

        double cmToInch = 0;
        public double inch = 0;
        //OpenCvCamera phoneCam;
        public static boolean RingStack;

        OpenCvCamera phoneCam;


        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

        LynxDcMotorController ctrl;
        LynxModule module;

        SkyStoneDriveBase skyStoneDriveBase;

        Trajectory trajectory;


        DropPositions position = DropPositions.B;

        @Override
        public void runOpMode() throws InterruptedException
        {

            trackingWheelIntegrator = new TrackingWheelIntegrator();

            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
            Globals.TSELift = hardwareMap.get(Servo.class, "TSELift");
            Globals.TSEClaw = hardwareMap.get(Servo.class, "TSEClaw");
            Globals.Booper = hardwareMap.get(Servo.class, "Dumper");
            Globals.RotationI = hardwareMap.get(Servo.class, "RotationI");
            Globals.DUCKwheel = hardwareMap.get(CRServo.class, "DUCKwheel");
            Globals.Lift = hardwareMap.get(DcMotorEx.class, "Lift");
            Globals.Intake = hardwareMap.get(DcMotor.class, "Intake");
            Globals.FrightDetector = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "REVDS");

            skyStoneDriveBase = new SkyStoneDriveBase();
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;



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
            inch = cmToInch/2.54;
           // telemetry.addData("Dist inch", -inch+2);
            //telemetry.addData("Position", RingPipline.GetPosition());
            telemetry.update();

            Globals.inch = inch;

            trackingWheelIntegrator.setFirstTrackingVal(-24,0);

            clearEnc();

            while (!isStopRequested() && !isStarted()) {
                position = TSEPipline.GetPosition();
                telemetry.addData("Position", TSEPipline.GetPosition());
                telemetry.addData("Dist inch", inch+1);
                telemetry.update();
            }
            if (position == DropPositions.A) { //Lvl 1
                CapstoneXPos= 71.6;
                CapstoneYPos= 15.5;
                CapstoneHeading= 180;
                CapstoneXPos2= 71.6;
                CapstoneYPos2= 17.7;
                CapstoneHeading2= 180;
                LiftPreLoad.Lvl = 130;
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
            buildTrajectory();
            trajectory.follow();
        }
        void clearEnc()        {
            ctrl.setMotorMode(0, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            ctrl.setMotorMode(0, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        public void buildTrajectory()
        {
            trajectory = new Trajectory.Builder()
                    .addMovement(new TSEDeploy())
                    .addMovement(new PointApproach.Builder() //BEGIN THE QUEST TO SAVE THE DUCK(moves to shipping hub)
                            .setTargetPosition(-38,32)
                            .setMaxPower(0.6)
                            .setXyGain(.06)
                            .setTargetHeading(-113)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new LiftPreLoad())
                    .addMovement(new PreLoadPlaceMineral())
                    .addMovement(new PointApproach.Builder() // PLACE MINERAL!!!!!!!!!!!!!!!!
                            .setTargetPosition(-10.5,8.5)
                            .setMaxPower(0.7001)
                            .setXyGain(.06)
                            .setTargetHeading(-133)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new PointApproach.Builder() // PLACE MINERAL!!!!!!!!!!!!!!!!
                            .setTargetPosition(-9.3,5.8)
                            .setMaxPower(0.3)
                            .setXyGain(.06)
                            .setTargetHeading(-133)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(.5)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new LiftDown())
                    .addMovement(new DuckyWheelBlue())
                    .addMovement(new SleepAction(1000))
                    .addMovement(new FFIntakeOn())
                    .addMovement(new PointApproach.Builder() // PLACE MINERAL!!!!!!!!!!!!!!!!
                            .setTargetPosition(-8,3.5)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-170)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    //.addMovement(new LiftPreLoad())
                    //.addMovement(new SleepAction(500))
                    //.addMovement(new PreLoadPlaceMineral())
                    //.addMovement(new FFIntakeOn()) //TURN ON THE DUCK SAVING MACHINE!!!!!!!!!!!!!!
                    .addMovement(new PointApproach.Builder() // PICK UP DUCKY(OH THE POOR DUCK)!!!!!!!!!
                            .setTargetPosition(-24,2)
                            .setMaxPower(0.6)
                            .setXyGain(.06)
                            .setTargetHeading(-175)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new LiftUp())
                    .addMovement(new PointApproach.Builder() //BEGIN THE QUEST TO SAVE THE DUCK(moves to shipping hub)
                            .setTargetPosition(-38,32)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-105)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new PlaceMineral())
                    .addMovement(new LiftDown())
                    .addMovement(new PointApproach.Builder() //BEGIN THE QUEST TO SAVE THE DUCK(moves to shipping hub)
                            .setTargetPosition(-32,56)
                            .setMaxPower(0.6)
                            .setXyGain(.06)
                            .setTargetHeading(-97)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new PointApproach.Builder() //BEGIN THE QUEST TO SAVE THE DUCK(moves to shipping hub)
                            .setTargetPosition(-72,54)
                            .setMaxPower(0.6)
                            .setXyGain(.06)
                            .setTargetHeading(-47)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new PointApproach.Builder() //BEGIN THE QUEST TO SAVE THE DUCK(moves to shipping hub)
                            .setTargetPosition(-78,22)
                            .setMaxPower(0.6)
                            .setXyGain(.06)
                            .setTargetHeading(-90)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())


                    /*

                    //.addMovement(new LiftDown())
                    .addMovement(new PointApproach.Builder() // SAVE THE DUCK PLEASE PICK HIM UP!!!!!!!!!!!!
                            .setTargetPosition(-9,5.3)
                            .setMaxPower(0.4)
                            .setXyGain(.06)
                            .setTargetHeading(-180)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new IntakeOff())//TURN OFF THE INTAKE SO AS TO NOT HARM HIS HIGHNESS'S EARS
                    .addMovement(new PointApproach.Builder() // RETURN THE GREAT AND POWERFUL DUCK TO HIS KINGDOM OF THE GOAL(timmy thinks it's the "shipping hub" the duck does not like it)
                            .setTargetPosition(-44,12)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-150)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new Transfer())
                    .addMovement(new PointApproach.Builder() // RETURN THE GREAT AND POWERFUL DUCK TO HIS KINGDOM OF THE GOAL(timmy thinks it's the "shipping hub" the duck does not like it)
                            .setTargetPosition(-46,24)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-150)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new LiftUp())//LIFT HIM TO HIS FAVOURITE ROOM AT THE VERY TOP
                    .addMovement(new SleepAction(500))
                    .addMovement(new PlaceMineral())//THE DUCK IS NOT A MINERAL(BUT HE LIKES BEING PUT IN HIS ROOM)

                    .addMovement(new PointApproach.Builder() //AFTER PLACING THE GREAT DUCK THE MACHINE PREPARES TO PLACE MANY GIFTS BEFORE THE DUCK
                            .setTargetPosition(-8,28)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(-180)
                            .setTargetHeading(-180)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new LiftDown())
                    .addMovement(new FFPrepareForTele())

                     */
                    .addMovement(new SleepAction(2000))
                    .build();

        }

}
