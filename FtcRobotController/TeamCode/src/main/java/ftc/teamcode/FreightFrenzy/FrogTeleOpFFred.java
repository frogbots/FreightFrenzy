package ftc.teamcode.FreightFrenzy;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.AUXLift;
import org.firstinspires.ftc.teamcode.trajectory.AutoLiftDownWithWait;
import org.firstinspires.ftc.teamcode.trajectory.AutoPlaceAndStow;
import org.firstinspires.ftc.teamcode.trajectory.AutoTransfer;
import org.firstinspires.ftc.teamcode.trajectory.LeftTWLift;
import org.firstinspires.ftc.teamcode.trajectory.RightTWLift;
import org.firstinspires.ftc.teamcode.trajectory.StateMTrajectory;

import ftc.teamcode.Toggler;

import static org.firstinspires.ftc.teamcode.Globals.ARML;
import static org.firstinspires.ftc.teamcode.Globals.ARMR;
import static org.firstinspires.ftc.teamcode.Globals.BackTouch;
import static org.firstinspires.ftc.teamcode.Globals.FrightDistance;
import static org.firstinspires.ftc.teamcode.Globals.LiftPos;
import static org.firstinspires.ftc.teamcode.Globals.RightTouch;
import static org.firstinspires.ftc.teamcode.Globals.TSEMotor;
import static org.firstinspires.ftc.teamcode.Globals.TSERotation;
import static org.firstinspires.ftc.teamcode.Globals.Turret;


@TeleOp
public class FrogTeleOpFFRED extends LinearOpMode {

//    static LynxDcMotorController ctrl;
//    LynxModule module;


    private DcMotorEx FL;
        private DcMotorEx FR;
        private DcMotorEx RL;
        private DcMotorEx RR;
        Toggler LiftUp = new Toggler();
        Toggler LiftDown = new Toggler();
        StateMTrajectory trajectory;
        AutoTransfer AUTOxferSm = new AutoTransfer();
        AutoPlaceAndStow AUTOPLACE = new AutoPlaceAndStow();
        AutoLiftDownWithWait AutoLiftDownWithWait = new AutoLiftDownWithWait();
        AUXLift AUXLift = new AUXLift();
        LeftTWLift LeftTWLift = new LeftTWLift();
        RightTWLift RightTWLift = new RightTWLift();
        SkyStoneDriveBase skyStoneDriveBase;
        private CRServo DUCKwheel;
        private CRServo RTW;
        private CRServo LTW;
        private CRServo ATW;
        int LiftAdjustment;
        public static boolean FrightDetection;
        public static boolean FrontDistance;
        long elapsedTime;
        int LiftPosition=0;
        int LiftHeight=3;
        TouchSensor LiftLimit;
        boolean BTouch;
        boolean RTouch;
        boolean LTouch;
        boolean LiftVsTSE;
        boolean OdoIsLifting = false;
        public static boolean TurretTurn = false;
        public static double TurretTARGET;
        public static double TurretPos;
        double Kp = 1.2; //2.2
        double Ki =.008; //.02
        double Kd =1;  //15
        double integral;
        double error;
        double turnPower;
        boolean MotorGo;
        double derivative;
        double lastError;
        double RealPot;
        double targetPosition;
        double TSER = .9;
        double TSEL;
        public static boolean LiftGoDown = false;
        private long stateStartTime;
        double ArmRight;
        double ArmLeft;
        public static double RotationIntake;





    AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl;
        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();


        static LynxDcMotorController ctrl;
        LynxModule module;

        @Override
        public void runOpMode() throws InterruptedException {


            trackingWheelIntegrator = new TrackingWheelIntegrator();

            //maping out the robot
            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
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
            //TSELift = hardwareMap.get(Servo.class, "TSELift");
            //TSEClaw = hardwareMap.get(Servo.class, "TSEClaw");
            //Booper = hardwareMap.get(Servo.class, "Dumper");
            //DumperTesting = hardwareMap.get(Servo.class, "DumperTesting");
            Globals.ARML = hardwareMap.get(Servo.class, "ARML");
            Globals.ARMR = hardwareMap.get(Servo.class, "ARMR");
            Globals.TSERotation = hardwareMap.get(Servo.class,  "TSERotation");
            Globals.TSELift = hardwareMap.get(Servo.class,  "TSELift");
            Globals.HEXCLAW = hardwareMap.get(Servo.class, "HEXCLAW");
            Globals.RotationI = hardwareMap.get(Servo.class, "RotationI");
            Globals.DUCKwheel = hardwareMap.get(CRServo.class, "DUCKwheel");
            Globals.RTW = hardwareMap.get(CRServo.class, "RTW");
            Globals.LTW = hardwareMap.get(CRServo.class, "LTW");
            Globals.ATW = hardwareMap.get(CRServo.class, "ATW");
            Globals.LeftTouch = hardwareMap.get(TouchSensor.class, "LeftTouch");
            Globals.RightTouch = hardwareMap.get(TouchSensor.class, "RightTouch");
            Globals.BackTouch = hardwareMap.get(TouchSensor.class, "BackTouch");
            Globals.Lift = hardwareMap.get(DcMotorEx.class, "Lift");
            Globals.leftTW = hardwareMap.get(DcMotorEx.class, "Intake"); // this is also left Tracking wheel
            Globals.rightTW = hardwareMap.get(DcMotorEx.class, "TSEMotor");
            Globals.backTW = hardwareMap.get(DcMotorEx.class, "Turret");
            Globals.Intake = hardwareMap.get(DcMotorEx.class, "Intake");
            Globals.Turret = hardwareMap.get(DcMotorEx.class, "Turret");
            Globals.TSEMotor = hardwareMap.get(DcMotorEx.class, "TSEMotor");
            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");
            Globals.FrightDetector = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "REVDS");
            //Globals.FrontDS = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "FrontDistance");
            // imu = new FrogBNO055( hardwareMap.get(BNO055IMU.class, "external_IMU"));
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            Globals.potentiometer = hardwareMap.get(AnalogInput.class, "Potentiometer");
            //Globals.Backsonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
            //Globals.LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
            //Globals.RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
            Globals.odoModule = module;
            Globals.opMode = this;
            Globals.robot.enableBrake(true);


            Globals.Lift.setDirection(DcMotorSimple.Direction.REVERSE);
            //Globals.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boolean AutomationLastState = false;
            acclCtrl = new AccerlationControlledDrivetrainPowerGeneratorForAuto(.08, 1, .05);

            telemetry.setMsTransmissionInterval(20);
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            Globals.Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //clearEnc();
            //imu.init();
            Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Globals.Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Globals.Lift.setTargetPosition(1);
            Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            TurretTARGET  = 1.29;
            TurretTurn =false;
            waitForStart();

            while (opModeIsActive()) {
                  //RotationIntake = getDouble("RI");
//                Kp = getDouble("Kp");
//                Ki = getDouble("Ki");
//                Kd = getDouble("Kd");
//                Globals.TurretTARGET = getDouble("Targetposition");
//                Globals.TurretTurn = getBoolean("MotorGo");

                //TSE Rotation In .9 ; Out .25
                //TSE Lift UP .6 Down; .95

                Globals.updateTracking();
                if (LiftGoDown) {         //If button is pressed Auto Movements will run. else GAMEPAD
                    if (AutomationLastState == false) {
                        //AutoLiftDownWithWaitTele.reset();

                    }
                    //AutoLiftDownWithWaitTele.runIteration();
                    runGamepad();
                    AutomationLastState = true;

                }
                else if (OdoIsLifting) {
                    if (AutomationLastState == false) {
                        AUXLift.reset();
                        LeftTWLift.reset();
                        RightTWLift.reset();
                    }
                    BTouch = BackTouch.isPressed();
                    RTouch = RightTouch.isPressed();
                    LTouch = Globals.LeftTouch.isPressed();
                    if (BTouch) {
                        AUXLift.runIteration();
                    }
                    if (LTouch) {
                        LeftTWLift.runIteration();
                    }
                    if (RTouch) {
                        RightTWLift.runIteration();
                    }
                    runGamepad();
                    AutomationLastState = true;

                }
                else if (FrightDetection) {
                   if (AutomationLastState == false) {
                       AUTOxferSm.reset();
                       gamepad1.rumbleBlips(2);
                    }
                    runGamepad();
                    AutomationLastState = true;
                    AUTOxferSm.runIteration();
                }
                else {
                    runGamepad();
                    if (AutomationLastState == true) {
                        AutomationLastState = false;
                        Globals.robot.enableBrake(true);
                        Globals.robot.disablePID();

                        //trajectory.reset();

                    }
                }
            }
        }

    void runGamepad() {


            Globals.TurretPos = Globals.currentVoltage;
            RealPot = Globals.TurretPos;
            if (FrightDistance < 73) {
                FrightDetection = true;
            }
            // .44 Right 90
            // 2.62 Left 90
            // 1.29 Stow
            //Positive is right (INTAKE = front)
            //Negative is left


//        if (Globals.TurretTurn) {
//            if (error < 0 && lastError > 0 || error > 0 && lastError < 0 ){
//                integral = 0;
//            }
//
//            error = Globals.TurretTARGET - RealPot;
//            integral = integral + error;
//            derivative = error - lastError;
//            turnPower = (Kp*(error) + Ki * (integral) + Kd * (derivative));
//            if (Math.abs(turnPower) > 1) {
//                integral = integral - error;
//            }
//            if ((RealPot > 2.8 && turnPower < 0)|| (RealPot < .34 && turnPower > 0)) {
//                //consider adding a Method to make it be able to tun back into range
//                TurretTurn = false;
//                Turret.setPower(0);
//            }
//            Globals.turnPower= turnPower;
//            if (Globals.TurretTurn) {
//                Turret.setPower(turnPower);
//            }
//            lastError = error;
//        }
//        else {
//            integral = 0;
//            Turret.setPower(0);
//        }

        if (Globals.TurretTurn) {
            if (error < 0 && lastError > 0 || error > 0 && lastError < 0 ){
                integral = 0;
            }

            error = Globals.TurretTARGET - RealPot;
            integral = integral + error;
            derivative = error - lastError;
            turnPower = (Kp*(error) + Ki * (integral) + Kd * (derivative));
            if (Math.abs(turnPower) > 1) {
                integral = integral - error;
            }
            if ((RealPot > 2.8 && turnPower < 0)|| (RealPot < .34 && turnPower > 0)) {
                Globals.TurretTurn = false;
                Turret.setPower(0);
            }
            if (Globals.TurretTurn) {
                Turret.setPower(turnPower);
            }
            lastError = error;
        }
        else {
            integral = 0;
            Turret.setPower(0);
        }
        if (gamepad2.left_stick_y <.1) {
            Globals.TSEPOWER =gamepad2.left_stick_y*gamepad2.left_stick_y;
            TSEMotor.setPower(-Globals.TSEPOWER);
        }
        else if (gamepad2.left_stick_y >-.1) {
            Globals.TSEPOWER =-gamepad2.left_stick_y*gamepad2.left_stick_y;
            TSEMotor.setPower(-Globals.TSEPOWER);
        }
        else {
            TSEMotor.setPower(0);
        }
        if (gamepad2.left_stick_x > .1){
            ArmRight = ArmRight + (gamepad2.right_stick_x/50);
            ArmLeft = ArmLeft - (gamepad2.right_stick_x/50);
            ARMR.setPosition(ArmRight);
            ARML.setPosition(ArmLeft);
        }
        else if (gamepad2.left_stick_x < -.1 ) {
            ArmRight = ArmRight - (gamepad2.right_stick_x/50);
            ArmLeft = ArmLeft + (gamepad2.right_stick_x/50);
            ARMR.setPosition(ArmRight);
            ARML.setPosition(ArmLeft);
        }
        if (ArmLeft < .3) { ArmLeft = .3;}
        if (ArmLeft > .15) { ArmLeft = .15;}
        if (ArmRight > .95) {ArmRight = .95;}
        if (ArmRight < .7) {ArmRight = .7;}


        Globals.turnPower=turnPower;
        if (TSEL > .95){TSEL = .95;}
        if (TSEL < .55){TSEL = .55;}
        if (TSER < .25){TSER = .35;}
        if (TSER > .9){TSER = .9;}
        Globals.TSELift.setPosition(TSEL);
        TSERotation.setPosition(TSER);
        if (gamepad2.right_bumper && gamepad2.left_bumper) {
                Globals.Lift.setPower(0);
            Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }


            if (gamepad2.right_stick_y > .1){
                TSEL = TSEL + (gamepad2.right_stick_y/50);
            }
            else if (gamepad2.right_stick_y < -.1 ) {
                TSEL = TSEL + (gamepad2.right_stick_y/50);
            }
            if (gamepad2.right_stick_x > .1){
                TSER = TSER + (gamepad2.right_stick_x/50);
            }
            else if (gamepad2.right_stick_x < -.1 ) {
                TSER = TSER + (gamepad2.right_stick_x/50);
            }

            if (gamepad2.dpad_down) {
                Globals.ARML.setPosition(.15);
                Globals.ARMR.setPosition(.85);
            }
            if (gamepad2.dpad_up) {

                Globals.TurretTARGET = 1.29;
                Globals.ARML.setPosition(.2);
                Globals.ARMR.setPosition(.8);
                Globals.TurretTurn = true;
            }
            if (gamepad2.dpad_left) {
                Globals.TurretTARGET = .88;
                Globals.ARML.setPosition(.2);
                Globals.ARMR.setPosition(.8);
                Globals.TurretTurn = true;
            }
            if (gamepad2.dpad_right) {
                Globals.TurretTARGET = 1.85;
                Globals.ARML.setPosition(.2);
                Globals.ARMR.setPosition(.8);
                Globals.TurretTurn = true;
            }

             /*if (OdoIsLifting == true) {
                if (BTouch) {
                    ATW.setPower(0);
                }
                if (RTouch) {
                    RTW.setPower(0);
                }
                if (LTouch) {
                    LTW.setPower(0);
                }
            }

              */
//            BTouch = BackTouch.isPressed();
//            RTouch = RightTouch.isPressed();
//            LTouch = Globals.LeftTouch.isPressed();
        if (gamepad1.right_trigger > .2) {Globals.DUCKwheel.setPower(-1);}
            else {Globals.DUCKwheel.setPower(0);}
            /*if (gamepad2.dpad_up) {
                OdoIsLifting  = true;
                LTW.setPower(1);
                //ATW.setPower(-1);
                //=RTW.setPower(1);
               /* sleep(700);
                ATW.setPower(.2);
                sleep(300);
                RTW.setPower(-.2);
                LTW.setPower(-.2);
                sleep(200);
                LTW.setPower(0);
                ATW.setPower(0);
                RTW.setPower(0);

                */
           // }


            MecanumDrive.cartesian(Globals.robot,
                    -gamepad1.left_stick_y, // Main
                    gamepad1.left_stick_x , // Strafe
                    gamepad1.right_stick_x * .85); // Turn

            if(LiftUp.shouldToggle(gamepad2.right_trigger > .5)){Globals.LiftLevel = Globals.LiftLevel+1;}
            if(LiftDown.shouldToggle(gamepad2.left_trigger > .5)){Globals.LiftLevel = Globals.LiftLevel-1;}
            if (gamepad1.a) {
                Globals.RotationI.setPosition(.12);
                Globals.Intake.setPower(1);
                FrightDetection = false;
                Globals.HEXCLAW.setPosition(.1);


            }
            if (gamepad1.b) {Globals.Intake.setPower(0); Globals.RotationI.setPosition(.83);}
            if (gamepad1.x) {Globals.Intake.setPower(-1);}
            if (gamepad1.left_bumper && !OdoIsLifting) {
                OdoIsLifting=true;
                LTW.setPower(0);                                                    // CHECK POWER TMR
                ATW.setPower(0);
                RTW.setPower(0);

            }

           if (gamepad2.x) {   //PLACE
                Globals.HEXCLAW.setPosition(.1);
                /*

                ARML.setPosition(.95);
                ARMR.setPosition(.05);
                LiftPosition = 1;
                Globals.LiftTarget = 80+LiftAdjustment;
                Globals.Lift.setTargetPosition(1);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(0.7);//(1+LiftAdjustment) - LiftPos)/400
                //Booper.setPosition(.5);



                 */

            }

            if (gamepad2.b) {   //STOW
                //LiftGoDown = true;
                Globals.HEXCLAW.setPosition(0);
                Globals.ARML.setPosition(.94);
                Globals.ARMR.setPosition(.06);
                LiftPosition = 1;
                Globals.LiftTarget = 0;
                Globals.Lift.setTargetPosition(10);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(0.7);
                //Globals.TurretTARGET = 1.34;
                Globals.TurretTARGET = 1.33;
                Globals.TurretTurn = true;
            }
            if (LiftPos == 0) {
                Globals.Lift.setPower(0);
                Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad2.a) {   //Allince Deploy
                Globals.ARML.setPosition(.2);
                Globals.ARMR.setPosition(.8);
                Globals.Intake.setPower(0);
                LiftPosition = 2;
                Globals.LiftTarget = LiftHeight;
                //Globals.LiftLevel = 3;
                Globals.Lift.setTargetPosition(LiftHeight);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);
            }
            if (gamepad2.y) {   //SHARED Deploy
                Globals.ARML.setPosition(.2);
                Globals.ARMR.setPosition(.8);
                Globals.Intake.setPower(0);
                LiftPosition = 2;
                Globals.LiftTarget = LiftHeight+LiftAdjustment;
                //Globals.LiftLevel = 3;
                Globals.Lift.setTargetPosition(20);
                Globals.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Globals.Lift.setPower(1);
                Globals.TurretTARGET = .5;
                Globals.TurretTurn = true;
            }
        if (!LiftVsTSE) {

            }
        if (Globals.LiftLimit.isPressed()) {
               Globals.Lift.setPower(0);
               Globals.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

            if (Globals.LiftLevel < 1) {Globals.LiftLevel = 1;}
            if (Globals.LiftLevel > 3) {Globals.LiftLevel = 3;}
            if (Globals.LiftLevel == 1) {LiftHeight = 00;}
            if (Globals.LiftLevel == 2) {LiftHeight = 80;}
            if (Globals.LiftLevel == 3) {LiftHeight = 330;}



    }
//    public void buildTrajectory() {
//
//        trajectory = new StateMTrajectory.Builder()
//                .addMovement(new AutoTransfer())
//                .build();
//    }
    public void AutoPlaceAndStow() {

        trajectory = new StateMTrajectory.Builder()
                .addMovement(new AutoPlaceAndStow())
                .build();
    }
    public long getElapsedStateTime()
    {
        return System.currentTimeMillis() - stateStartTime;
    }
        public static void clearEnc()        {
        ctrl.setMotorMode(3, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        ctrl.setMotorMode(3, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
