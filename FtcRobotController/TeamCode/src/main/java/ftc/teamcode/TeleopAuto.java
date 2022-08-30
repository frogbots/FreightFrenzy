package ftc.teamcode;/*  package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.MovingStatistics;

import control.AcceleratedGain;
import robotComponents.drivebase.SkyStoneDriveBase;
import trajectory.PointApproach;
import trajectory.SleepAction;
import trajectory.Trajectory;
import trajectory.Waypoint;

@Override
public void run() {
{
    MovingStatistics movingStatistics = new MovingStatistics(300);

    long startLoop = 0;

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

        clearEnc();

        telemetry.setMsTransmissionInterval(50);

        Globals.driveBase = skyStoneDriveBase;
        Globals.opMode = this;
        Globals.trackingWheelIntegrator = trackingWheelIntegrator;
        Globals.odoModule = module;

        buildTrajectory();

        waitForStart();

        trajectory.follow();
    }


    void clearEnc()
    {
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

                //
                // * Collect first
                //
                .addMovement(new PointApproach.Builder()
                        .setTargetPosition(18,23.7)
                        .setMaxPower(0.35)
                        .setXyGain(.04)
                        .setTargetHeading(90)
                        .setHeadingDynamicGain(new AcceleratedGain(.012, 0.0004))
                        .setMaxTurnPower(0.45)
                        .setMovementThresh(1)
                        .setHeadingThreshold(0)
                        .stopMotorsOnDone(true)
                        .build())
                .addMovement(new SleepAction(500)).build();
    }
}


 */

