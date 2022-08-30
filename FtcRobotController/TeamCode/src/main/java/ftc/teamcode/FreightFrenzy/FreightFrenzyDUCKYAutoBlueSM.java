package ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.control.AcceleratedGain;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.AUTOLiftPreLoad;
import org.firstinspires.ftc.teamcode.trajectory.AUTOLiftPreLoadDuck;
import org.firstinspires.ftc.teamcode.trajectory.AutoDuckyWheel;
import org.firstinspires.ftc.teamcode.trajectory.AutoLiftDownWithWait;
import org.firstinspires.ftc.teamcode.trajectory.AutoLiftUp;
import org.firstinspires.ftc.teamcode.trajectory.AutoPlaceAndStow;
import org.firstinspires.ftc.teamcode.trajectory.AutoTransfer;
import org.firstinspires.ftc.teamcode.trajectory.DropMineral;
import org.firstinspires.ftc.teamcode.trajectory.IntakeOnSM;
import org.firstinspires.ftc.teamcode.trajectory.LiftPreLoad;
import org.firstinspires.ftc.teamcode.trajectory.NoDuckPark;
import org.firstinspires.ftc.teamcode.trajectory.ReadyToPlace;
import org.firstinspires.ftc.teamcode.trajectory.StateMPointApproach;
import org.firstinspires.ftc.teamcode.trajectory.StateMTrajectory;
import org.firstinspires.ftc.teamcode.trajectory.TurretSMAUTO;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import ftc.teamcode.DropPositions;

import static ftc.teamcode.FreightFrenzy.FrogTeleOpFF.TurretTARGET;
import static ftc.teamcode.FreightFrenzy.FrogTeleOpFF.TurretTurn;
import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.Cycle;
import static org.firstinspires.ftc.teamcode.Globals.GoDucks;
import static org.firstinspires.ftc.teamcode.Globals.HEXCLAW;
import static org.firstinspires.ftc.teamcode.Globals.RotationI;
import static org.firstinspires.ftc.teamcode.Globals.TRANSFERGoing;
import static org.firstinspires.ftc.teamcode.Globals.TSELift;
import static org.firstinspires.ftc.teamcode.Globals.Turret;
import static org.firstinspires.ftc.teamcode.Globals.WeHaveTheGoods;

@Autonomous(preselectTeleOp="FrogTeleOpFF")
public class FreightFrenzyDUCKYAutoBlueSM extends LinearOpMode {
        MovingStatistics movingStatistics = new MovingStatistics(300);

        long startLoop = 0;


        double GapHeading;
        double GapY1;
        double GapY2;
        double GapY3;
        AutoLiftUp AutoLiftUp = new AutoLiftUp();
        AUTOLiftPreLoad AUTOLiftPreLoad = new AUTOLiftPreLoad();
        AUTOLiftPreLoadDuck AUTOLiftPreLoadDuck = new AUTOLiftPreLoadDuck();
        AutoPlaceAndStow AutoPlaceAndStow = new AutoPlaceAndStow();
        AutoTransfer AutoTransfer = new AutoTransfer();
        AutoLiftDownWithWait AutoLiftDownWithWait = new AutoLiftDownWithWait();
        DropMineral DropMineral = new DropMineral();
        TurretSMAUTO TurretSMAUTO = new TurretSMAUTO();
        double cmToInch = 0;
        public double inch = 0;
        //OpenCvCamera phoneCam;
        public static boolean RingStack;
        double Kp=2; //1.2
        double Ki=.01; //.008
        double Kd=2.5; //1
        OpenCvCamera phoneCam;


        double integral;
        double error;
        double turnPower;
        boolean MotorGo;
        double derivative;
        double lastError;
        double RealPot;
        double targetPosition;
        double GAPAdjustment;


        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

        static LynxDcMotorController ctrl;
        LynxModule module;

        SkyStoneDriveBase skyStoneDriveBase;

        StateMTrajectory trajectory;

        StateMTrajectory FirstMovment;
        StateMTrajectory HubToDuckPrison;
        StateMTrajectory Rescue;
        StateMTrajectory FieldsToTower;
        StateMTrajectory TowerToViewingPoint;
        StateMTrajectory Park;

        StateMTrajectory LiftUpPreLoad;
        StateMTrajectory LiftDown;
        StateMTrajectory Xfer;


    DropPositions position = DropPositions.B;

        @Override
        public void runOpMode() throws InterruptedException
        {

            trackingWheelIntegrator = new TrackingWheelIntegrator();

            //Globals.FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
            Globals.HEXCLAW = hardwareMap.get(Servo.class, "HEXCLAW");
            Globals.ARML = hardwareMap.get(Servo.class, "ARML");
            Globals.ARMR = hardwareMap.get(Servo.class, "ARMR");
            TSELift = hardwareMap.get(Servo.class, "TSELift");
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
            Globals.rightTW = hardwareMap.get(DcMotorEx.class, "TSEMotor");
            Globals.backTW = hardwareMap.get(DcMotorEx.class, "Turret");
            Globals.Turret = hardwareMap.get(DcMotorEx.class, "Turret");
            Globals.potentiometer = hardwareMap.get(AnalogInput.class, "Potentiometer");
            //Globals.FrontDS = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "FrontDistance");
            Globals.LiftLimit = hardwareMap.get(TouchSensor.class, "LiftLimit");

            skyStoneDriveBase = new SkyStoneDriveBase();
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;

            Turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Globals.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Globals.Lift.setTargetPosition(1);
            Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Globals.Lift.setDirection(DcMotorSimple.Direction.REVERSE);


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



            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Position", TSEPipline.GetPosition());
            telemetry.update();

            Globals.inch = inch;

            trackingWheelIntegrator.setFirstTrackingVal(0,24);

            //TurretTARGET=1.29;
            GapHeading = -270;
            GapY1 = -.5;
            GapY2 = -.5;
            GapY3 = -.5;
            Globals.FirstMoving = true;


            clearEnc();
            position = DropPositions.C;

            while (!isStopRequested() && !isStarted()) {
                position = TSEPipline.GetPosition();
                telemetry.addData("Position", TSEPipline.GetPosition());
                //telemetry.addData("Dist inch", inch+1);
                telemetry.update();
                HEXCLAW.setPosition(0);
                ARML.setPosition(.6);
                ARMR.setPosition(.4);
                RotationI.setPosition(.6);

            }
            if (position == DropPositions.A) { //Lvl 1
                LiftPreLoad.Lvl = 1;
                Globals.LOWERARM = true;
            }
            if (position == DropPositions.B) { // Lvl 2
                LiftPreLoad.Lvl = 80;
                Globals.LOWERARM = false;
            }
            if (position == DropPositions.C) { //Lvl 3
                LiftPreLoad.Lvl = 340;
                Globals.LOWERARM = false;
            }
            Globals.HEXCLAW.setPosition(0);

            buildTrajectory();
            TurretTARGET = 1.29;
            Globals.ReadyToPlace = false;
            Globals.NoDuckPark= false;

            while (opModeIsActive()) {
                TSELift.setPosition(.5);

                Globals.TurretPos = Globals.currentVoltage;
                RealPot = Globals.TurretPos;

                if (Globals.TurretPos == Globals.TurretTARGET) {
                    Globals.TurretTurn = false;
                    Turret.setPower(0);
                }
                if (Globals.TurretTurn) {

                    if (error < 0 && lastError > 0 || error > 0 && lastError < 0) {
                        integral = 0;
                    }
                    error = Globals.TurretTARGET - RealPot;
                    integral = integral + error;
                    derivative = error - lastError;
                    turnPower = (Kp * (error) + Ki * (integral) + Kd * (derivative));
                    if (Math.abs(turnPower) > 1) {
                        integral = integral - error;
                    }
                    Globals.turnPower = turnPower;
                    if ((RealPot > 2.8 && turnPower < 0) || (RealPot < .34 && turnPower > 0)) {
                        Globals.TurretTurn = false;
                        Turret.setPower(0);
                    }
                    if (Globals.TurretTurn) {
                        Turret.setPower(turnPower);
                    }
                    lastError = error;

                    Turret.setPower(turnPower);
                }
                else {
                    integral = 0;
                    Turret.setPower(0);
                }

                if (Globals.LiftLimit.isPressed()) {
                    Globals.Lift.setPower(0);
                    Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                Globals.updateTracking();
                if (Globals.Cycle > Globals.ResetCycle) {
                    AutoTransfer.reset();
                    AutoLiftDownWithWait.reset();

                    Globals.ResetCycle = Globals.ResetCycle +1;
                }
                if (Globals.FirstMoving) {
                    Globals.TurretTARGET = 1.29;
                    Globals.Cycle = 1;
                    Globals.ResetCycle = 2;
                    FirstMovment.followInteration();
                    AUTOLiftPreLoadDuck.runIteration();
                    GoDucks = true;
                }
                if (Globals.Cycle == 2 && Globals.FrightDistance > 80 && !Globals.WeHaveTheGoods && Globals.GoDucks) {
                    TurretTurn = true;
                    TurretTARGET =1.29;
                    AutoLiftDownWithWait.runIteration();
                    HubToDuckPrison.followInteration();
                    Globals.ReadyToPlace = false;

                }
                if (Globals.Cycle == 2 && Globals.FrightDistance > 80 && !Globals.WeHaveTheGoods && !Globals.GoDucks) {
                    Rescue.followInteration();
                }
                if (Globals.NoDuckPark && Globals.FrightDistance >80) {
                    Park.followInteration();
                }
                if (!Globals.FirstMoving && Globals.FrightDistance < 80) {
                    Globals.WeHaveTheGoods = true;
                    Globals.MineralInClaw = true;
                }
                if(!Globals.FirstMoving && Globals.WeHaveTheGoods) {

                    FieldsToTower.followInteration();
                    AutoTransfer.runIteration();
                    if (!TRANSFERGoing) {
                        //HEXCLAW.setPosition(0);
                        Globals.ARML.setPosition(.2);
                        Globals.ARMR.setPosition(.8);
                        Globals.Intake.setPower(0);
                        Globals.LiftTarget = 330;
                        //Globals.LiftLevel = 3;
                        Globals.Lift.setTargetPosition(330);
                        Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Globals.Lift.setPower(1);
                        if (Globals.LiftPos >200) {
                            TurretTurn = true;
                            TurretTARGET  = .65;
                        }
                        if (Globals.ReadyToPlace) {
                            HEXCLAW.setPosition(.1);
                            WeHaveTheGoods = false;
                            Cycle = 3;
                        }
                    }
                }
                if (Globals.Cycle == 3 && Globals.FrightDistance > 80 && !Globals.WeHaveTheGoods) {
                    TowerToViewingPoint.followInteration();
                    AutoLiftDownWithWait.runIteration();
                }
            }
            /*
            while (FirstMoving) {
                Globals.updateTracking();
                FirstMovment.followInteration();
                AUTOLiftPreLoad.runIteration();
            }

            while (!FirstMoving && Globals.FrightDistance > 80) {
                Globals.updateTracking();
                HubToHouse.followInteration();
                AutoPlaceAndStow.runIteration();
            }
            if (Globals.FrightDistance > 80) {
                WeHaveTheGoods = true;
            }

            while (WeHaveTheGoods) {
                Globals.updateTracking();
                HouseToHub.followInteration();
                AutoTransfer.runIteration();
                if (Globals.TRANSFERGoing = false) {
                    AutoLiftUp.runIteration();
                }
            }


             */


        }
        public static void clearEnc()        {
            ctrl.setMotorMode(3, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            ctrl.setMotorMode(3, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    public void buildTrajectory() {



        FirstMovment = new StateMTrajectory.Builder()

                .addMovement(new StateMPointApproach.Builder() //Shipping Hub movment
                        .setTargetPosition(45,5)
                        .setMaxPower(.6)
                        .setXyGain(.06)
                        .setTargetHeading(-90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(4)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //Shipping Hub movment
                        .setTargetPosition(38,34)
                        .setMaxPower(.4)
                        .setXyGain(.06)
                        .setTargetHeading(-180)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(4)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new ReadyToPlace())
                .build();


        HubToDuckPrison = new StateMTrajectory.Builder()


                .addMovement(new StateMPointApproach.Builder() //To wall
                        .setTargetPosition(42,7)
                        .setMaxPower(.5)
                        .setXyGain(.06)
                        .setTargetHeading(90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(5)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //To Duck
                        .setTargetPosition(15,7)
                        .setMaxPower(.5)
                        .setXyGain(.06)
                        .setTargetHeading(125)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(6)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //To Duck
                        .setTargetPosition(9.4,6)
                        .setMaxPower(.3)
                        .setXyGain(.06)
                        .setTargetHeading(125)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(2)
                        .setHeadingThreshold(10)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new AutoDuckyWheel())

                .build();


        Rescue = new StateMTrajectory.Builder()

                .addMovement(new IntakeOnSM())
                .addMovement(new StateMPointApproach.Builder() //Start OUT
                        .setTargetPosition(9.5,9)
                        .setMaxPower(.5)
                        .setXyGain(.04)
                        .setTargetHeading(90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(3)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //Out
                        .setTargetPosition(7,9)
                        .setMaxPower(.7)
                        .setXyGain(.06)
                        .setTargetHeading(90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(8)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //Out
                        .setTargetPosition(7,20)
                        .setMaxPower(.7)
                        .setXyGain(.06)
                        .setTargetHeading(90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(8)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new NoDuckPark())
                .build();

        FieldsToTower = new StateMTrajectory.Builder()


                .addMovement(new StateMPointApproach.Builder() //Shipping Hub movment
                        .setTargetPosition(44,6)
                        .setMaxPower(.5)
                        .setXyGain(.06)
                        .setTargetHeading(-90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(4)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //Shipping Hub movment
                        .setTargetPosition(37,33)
                        .setMaxPower(.4)
                        .setXyGain(.06)
                        .setTargetHeading(-180)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(2)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new ReadyToPlace())
                .build();


        TowerToViewingPoint = new StateMTrajectory.Builder()


                .addMovement(new StateMPointApproach.Builder() //Start OUT
                        .setTargetPosition(44,6)
                        .setMaxPower(.5)
                        .setXyGain(.04)
                        .setTargetHeading(-180)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(3)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new StateMPointApproach.Builder() //Out
                        .setTargetPosition(32,4)
                        .setMaxPower(.7)
                        .setXyGain(.06)
                        .setTargetHeading(-180)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(8)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .build();


        Park = new StateMTrajectory.Builder()


                .addMovement(new StateMPointApproach.Builder() //Out
                        .setTargetPosition(32,4)
                        .setMaxPower(.7)
                        .setXyGain(.06)
                        .setTargetHeading(-180)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(1)
                        .setMovementThresh(8)
                        .setHeadingThreshold(2)
                        .stopMotorsOnDone(true)
                        .build())
                .build();



//
//                .addMovement(new StateMWaypoint.Builder() //Movment To Gap
//                        .setTargetPosition(100, 3)
//                        .setTargetHeading(-94)
//                        .setSpeed(1)
//                        .setTransThreshMethod(StateMWaypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(1)
//                        .build())
//                .addMovement(new StateMWaypoint.Builder() //Movment To Gap
//                        .setTargetPosition(92, 4)
//                        .setTargetHeading(-82)
//                        .setSpeed(1)
//                        .setTransThreshMethod(StateMWaypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(1)
//                        .build())
//                .addMovement(new StateMWaypoint.Builder() //Movment To Gap
//                        .setTargetPosition(81, -2.5)
//                        .setTargetHeading(-80)
//                        .setSpeed(1)
//                        .setTransThreshMethod(StateMWaypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(1)
//                        .build())
//                .addMovement(new StateMWaypoint.Builder() //Movment To Gap
//                        .setTargetPosition(70, -11)
//                        .setTargetHeading(-26)
//                        .setSpeed(1)
//                        .setTransThreshMethod(StateMWaypoint.TranslationThreshMethod.Y_ONLY)
//                        .setMovementThresh(1)
//                        .setHeadingThreshold(1)
//                        .build())
//                /*
//                .addMovement(new StateMPointApproach.Builder()// BACKOUT
//                        .setTargetPosition(-105,GapY1)
//                        .setMaxPower(0.4)
//                        .setXyGain(.06)
//                        .setTargetHeading(GapHeading+2)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
//                        .setMaxTurnPower(0.5)
//                        .setMovementThresh(.5)
//                        .setHeadingThreshold(1)
//                        .stopMotorsOnDone(false)
//                        .build())
//                .addMovement(new StateMPointApproach.Builder()// ALL OUT
//                        .setTargetPosition(-74,GapY1)
//                        .setMaxPower(0.5)
//                        .setXyGain(.06)
//                        .setTargetHeading(-270)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
//                        .setMaxTurnPower(0.5)
//                        .setMovementThresh(.5)
//                        .setHeadingThreshold(1)
//                        .stopMotorsOnDone(false)
//                        .build())
//
//                 */
//                .addMovement(new StateMPointApproach.Builder()// GO TO HUB
//                        .setTargetPosition(65,21)
//                        .setMaxPower(1)
//                        .setXyGain(.06)
//                        .setTargetHeading(-32)
//                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
//                        .setMaxTurnPower(1)
//                        .setMovementThresh(3)
//                        .setHeadingThreshold(5)
//                        .stopMotorsOnDone(true)
//                        .build())
//                .addMovement(new PlaceMineralSM())
                //.build();

        /*
        trajectory = new StateMTrajectory.Builder()


                .addMovement(new StateMPointApproach.Builder() //Shipping Hub movment
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
           //Lift PreloAd
                .addMovement(new StateMPointApproach.Builder() //Shipping Hub movment
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
                //.addMovement(new PreLoadPlaceMineral()) // PLACE ONE
               // .addMovement(new LiftDown())

                .addMovement(new StateMPointApproach.Builder() //ENTER FOR TWO
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
                //.addMovement(new FFIntakeOn())

                .addMovement(new StateMPointApproach.Builder()// MOVE INTO FOR TWO
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
                .addMovement(new StateMPointApproach.Builder()// MOVE SLOWLY FOR PICKUP
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
                //.addMovement(new IntakeOff())
                .addMovement(new StateMPointApproach.Builder()// BACKOUT
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
                //.addMovement(new Transfer())
                .addMovement(new StateMPointApproach.Builder()// ALL OUT
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
                //.addMovement(new LiftUp())
                .addMovement(new StateMPointApproach.Builder()// GO TO HUB
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
                .addMovement(new StateMPointApproach.Builder()// CLOSER TO HUB SLOWLY
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
                .addMovement(new StateMPointApproach.Builder() //ENTER FOR THREE
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
                //.addMovement(new FFIntakeOn())
                .addMovement(new StateMPointApproach.Builder()// MOVE INTO FOR THREE
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
                .addMovement(new StateMPointApproach.Builder()// MOVE SLOWLY FOR PICKUP
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
                //.addMovement(new IntakeOff())
                .addMovement(new StateMPointApproach.Builder()// BACKOUT
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
                //.addMovement(new Transfer())
                .addMovement(new StateMPointApproach.Builder()// ALL OUT
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
                //.addMovement(new LiftUp())
                .addMovement(new StateMPointApproach.Builder()// GO TO HUB
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
                .addMovement(new StateMPointApproach.Builder()// CLOSER TO HUB SLOWLY
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
                .addMovement(new StateMPointApproach.Builder() //ENTER FOR PARK
                        .setTargetPosition(-74,GapY1)
                        .setMaxPower(0.8)
                        .setXyGain(.06)
                        .setTargetHeading(GapHeading)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(0.5)
                        .setMovementThresh(3)
                        .setHeadingThreshold(1)
                        .stopMotorsOnDone(true)
                        .build())
                //.addMovement(new FFIntakeOn())
                .addMovement(new StateMPointApproach.Builder()// MOVE INTO FOR PARK
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



                .build();


         */

    }



}



/*.addMovement(new StateMWaypoint.Builder() // First Stright movment to Target position
                        .setTargetPosition(25, 37)
                        .setTargetHeading(-40)
                        .setSpeed(1)
                        .setTransThreshMethod(StateMWaypoint.TranslationThreshMethod.Y_ONLY)
                        .setMovementThresh(1)
                        .setHeadingThreshold(0)
                        .build())

 */
