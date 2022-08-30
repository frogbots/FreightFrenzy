package org.firstinspires.ftc.teamcode.NotFF;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import org.firstinspires.ftc.teamcode.control.AcceleratedGain;
import org.firstinspires.ftc.teamcode.drivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;
import org.firstinspires.ftc.teamcode.trajectory.CapstonePickup;
import org.firstinspires.ftc.teamcode.trajectory.DuckyWheel;
import org.firstinspires.ftc.teamcode.trajectory.FFIntakeOn;
import org.firstinspires.ftc.teamcode.trajectory.FFPrepareForTele;
import org.firstinspires.ftc.teamcode.trajectory.FreightDetection;
import org.firstinspires.ftc.teamcode.trajectory.PlaceMineral;
import org.firstinspires.ftc.teamcode.trajectory.PointApproach;
import org.firstinspires.ftc.teamcode.trajectory.RelocationInWarehouse;
import org.firstinspires.ftc.teamcode.trajectory.SleepAction;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;

@Autonomous(preselectTeleOp="FrogTeleOp")
public class MainAuto extends LinearOpMode {
        MovingStatistics movingStatistics = new MovingStatistics(300);

        long startLoop = 0;

        double CapstoneXPos1;
        double CapstoneXPos2;
        double CapstoneXPos3;
        double CapstoneXPos = 20;

        double cmToInch = 0;
        public double inch = 0;
        //OpenCvCamera phoneCam;
        public static boolean RingStack;


        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

        LynxDcMotorController ctrl;
        LynxModule module;

        SkyStoneDriveBase skyStoneDriveBase;

        Trajectory trajectory;



        @Override
        public void runOpMode() throws InterruptedException
        {

            trackingWheelIntegrator = new TrackingWheelIntegrator();

            module = (LynxModule) hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            ctrl = hardwareMap.get(LynxDcMotorController.class, "Expansion Hub 3");

            skyStoneDriveBase = new SkyStoneDriveBase();
            skyStoneDriveBase.init(hardwareMap);
            skyStoneDriveBase.resetEncoders();
            skyStoneDriveBase.enableBrake(true);
            skyStoneDriveBase.enablePID();
            Globals.robot=skyStoneDriveBase;
            Globals.driveBase=skyStoneDriveBase;



            Globals.RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
            Globals.opMode = this;
            Globals.trackingWheelIntegrator = trackingWheelIntegrator;
            Globals.odoModule = module;

            //Globals.RingDetector = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "REVCS");

            telemetry.setMsTransmissionInterval(20);
            cmToInch = Globals.RightSonar.getDistanceSync();
            inch = cmToInch/2.54;
            telemetry.addData("Dist inch", -inch+2);
            //telemetry.addData("Position", RingPipline.GetPosition());
            telemetry.update();

            Globals.inch = inch;

            trackingWheelIntegrator.setFirstTrackingVal(-inch,0);

            clearEnc();

            while (!isStopRequested() && !isStarted()) {
                //position = RingPipline.GetPosition();
                //telemetry.addData("Position", RingPipline.GetPosition());
                telemetry.addData("Dist inch", inch+1);
                telemetry.update();
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


                    .addMovement(new PointApproach.Builder() //First movment to Ducky Wheel
                            .setTargetPosition(0,12)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(90) //check number
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new DuckyWheel())
                    .addMovement(new PointApproach.Builder() //Move to Pickup capstone
                            .setTargetPosition(-CapstoneXPos,20)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(90)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new CapstonePickup())
                    .addMovement(new PointApproach.Builder() //Move to Shipping Hub dropoff
                            .setTargetPosition(-56,20)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(180)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new PlaceMineral())
                    .addMovement(new PointApproach.Builder() //Move to warehouse
                            .setTargetPosition(-92,19)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(90)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(true)
                            .build())
                    .addMovement(new RelocationInWarehouse()) //includes movement to pipes
                    //need IMU here^
                    .addMovement(new FFIntakeOn())
                    .addMovement(new PointApproach.Builder() //Movement In Warehouse
                            .setTargetPosition(-116,10)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(135)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    .addMovement(new FreightDetection())
                    .addMovement(new PointApproach.Builder() //Move to Leave
                            .setTargetPosition(-100,19)
                            .setMaxPower(0.5)
                            .setXyGain(.06)
                            .setTargetHeading(135)
                            .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                            .setMaxTurnPower(0.6)
                            .setMovementThresh(1)
                            .setHeadingThreshold(2)
                            .stopMotorsOnDone(false)
                            .build())
                    //.addMovement(new BounceBackPSOneWithNoWobbleGoal())
                    .addMovement(new FFPrepareForTele())
                    .addMovement(new SleepAction(200))
                    .build();
        }

}
