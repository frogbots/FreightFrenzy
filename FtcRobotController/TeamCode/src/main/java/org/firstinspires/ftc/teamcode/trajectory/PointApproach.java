package org.firstinspires.ftc.teamcode.trajectory;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;
import org.firstinspires.ftc.teamcode.Vector;

import org.firstinspires.ftc.teamcode.control.AcceleratedGain;
import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.MotorPowers;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.SkyStoneDriveBase;

public class PointApproach implements MovementPerformer
{
    double targetX;
    double targetY;
    double maxPower;
    double targetHeading;
    AcceleratedGain acclHeadingGain;
    double maxTurnPower;
    double thresh;
    double angleThresh;
    double headingGain;
    double xyGain;
    boolean stopMotorsOnDone = false;
    TrackingWheelIntegrator trackingWheelIntegrator;
    SkyStoneDriveBase skyStoneDriveBase;

    public PointApproach()
    {
        trackingWheelIntegrator = Globals.trackingWheelIntegrator;
        skyStoneDriveBase = Globals.robot;
        //skyStoneDriveBase = Globals.driveBase;
    }

    @Override
    public void run()
    {
        moveToWaypoint();

        if(stopMotorsOnDone)
        {
            skyStoneDriveBase.stopMotors();
        }
    }

    void moveToWaypoint()
    {
        double error = 5;

        boolean angleGood = false;

        while ((Math.abs(error) > thresh || !angleGood) && opModeIsActive())
        {
            updateTracking();

            double turnError = trackingWheelIntegrator.getHeading() - targetHeading;

            double xErr = targetX - trackingWheelIntegrator.getX();
            double yErr = targetY - trackingWheelIntegrator.getY();

            if(angleThresh == 0)
            {
                angleGood = true;
            }
            else
            {
                angleGood = Math.abs(turnError) < angleThresh;
            }

            Vector driveVector = new Vector();
            driveVector.addCartesian(xErr, yErr);
            driveVector.rotateDegrees(-trackingWheelIntegrator.getHeading());

            driveVector.setCartesian(driveVector.getX()*1.5, driveVector.getY());

            double mag = Range.clip(driveVector.getMag()*xyGain, -maxPower, maxPower);

            double turnCorrection = Range.clip(turnError*getHeadingGain(), -maxTurnPower, maxTurnPower);

            MotorPowers pows = MecanumDrive.calcPolar(mag, driveVector.getDir(), turnCorrection);

            error = Vector.calcMag(xErr, yErr);

            skyStoneDriveBase.setMotorPowers(pows);
        }
    }

    double getHeadingGain()
    {
        if(acclHeadingGain == null)
        {
            return headingGain;
        }
        else
        {
            return acclHeadingGain.getControlledGain();
        }
    }

    boolean opModeIsActive()
    {
        return Globals.opMode.opModeIsActive();
    }

    void updateTracking()
    {
        Globals.updateTracking();
    }

    public static class Builder
    {
        double targetX;
        double targetY;
        double maxPower = .5;
        double targetHeading;
        AcceleratedGain acclHeadingGain;
        double maxTurnPower;
        double thresh;
        double angleThresh;
        double xyGain = 0.06;
        double headingGain = .012;
        boolean stopMotorsOnDone = false;

        public Builder setTargetPosition(double x, double y)
        {
            this.targetX = x;
            this.targetY = y;

            return this;
        }

        public Builder setMaxPower(double maxPower)
        {
            this.maxPower = maxPower;
            return this;
        }

        public Builder setXyGain(double xyGain)
        {
            this.xyGain = xyGain;
            return this;
        }

        public Builder setHeadingDynamicGain(AcceleratedGain headingGain)
        {
            this.acclHeadingGain = headingGain;
            return this;
        }

        public Builder setMaxTurnPower(double maxTurnPower)
        {
            this.maxTurnPower = maxTurnPower;
            return this;
        }

        public Builder setMovementThresh(double thresh)
        {
            this.thresh = thresh;
            return this;
        }

        public Builder setTargetHeading(double heading)
        {
            this.targetHeading = heading;
            return this;
        }

        public Builder stopMotorsOnDone(boolean stopMotorsOnDone)
        {
            this.stopMotorsOnDone = stopMotorsOnDone;
            return this;
        }

        public Builder setHeadingThreshold(double headingThreshold)
        {
            this.angleThresh = headingThreshold;
            return this;
        }

        public PointApproach build()
        {
            PointApproach pointApproach = new PointApproach();
            pointApproach.targetX          = targetX;
            pointApproach.targetY          = targetY;
            pointApproach.targetHeading    = targetHeading;
            pointApproach.maxPower         = maxPower;
            pointApproach.acclHeadingGain  = acclHeadingGain;
            pointApproach.maxTurnPower     = maxTurnPower;
            pointApproach.angleThresh      = angleThresh;
            pointApproach.headingGain      = headingGain;
            pointApproach.stopMotorsOnDone = stopMotorsOnDone;
            pointApproach.thresh           = thresh;
            pointApproach.xyGain           = xyGain;

            return pointApproach;
        }
    }
}
