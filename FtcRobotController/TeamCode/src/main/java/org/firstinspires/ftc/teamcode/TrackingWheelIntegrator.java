package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;


public class TrackingWheelIntegrator
{
    private double x, y;

    private double oldX, oldY;

    private int lastPosLeft, lastPosRight, lastPosAux;

    private int dPosLeft, dPosRight, dPosAux;

    double dH;

    int cPosLeft;
    int cPosRight;
    int cPosAux;


    private double dX, dY, heading;
    Vector trackingVector = new Vector(); //296.90372 //288.17579 //288.6475478 //288.9193056 worked //290.67
    Vector instantVector = new Vector();
    private double TICKS_PER_DEGREE = -290; //(-401.6311) 24 inch BASE. (-288.17579) Undershot 19 degrees off for 10 spins 13x15 BASE (-288.9193056) Under shoot
    private double TICKS_PER_INCH_NORMAL = 1730.85; //1713.75
    private double TICKS_PER_INCH_STRAFE = 1733.25; //1702.84
    double AUX_SPIN_SLIP = 0.0980175328943; // 0.101919839;  ?

    boolean init = false;

    boolean useImu = false;

    double wheelHead;

    double lastImu;

    MovingStatistics speedometer = new MovingStatistics(50);

    long lastUpdateTime;

    double headingOffsetDegrees;

    public void setFirstTrackingVal(double x, double y) {
        trackingVector.setCartesian(x,y);
            }
    public void setHeading(double newHeading) {
           heading = newHeading;


    }
    public void SetX(double NewX) {
        x = NewX;
        trackingVector.setCartesian(x,y);
    }
    public void SetY(double NewY) {
        y = NewY;
        trackingVector.setCartesian(x,y);
    }
    public void ZeroHeading() {
        lastPosLeft = cPosLeft;
        lastPosRight = cPosRight;
    }


    public void setHeadingOffsetDegrees(double imuHeading)
    {
        this.headingOffsetDegrees = heading - imuHeading ;

        System.out.println(String.format("Sync heading w/ IMU... IMU says %f wheels say %f offset is %f", imuHeading, heading, headingOffsetDegrees));
    }

    public void update(int cPosLeft, int cPosRight, int cPosAux)
    {
        if(!init)
        {
            init = true;
            lastPosLeft = cPosLeft;
            lastPosRight = cPosRight;
            lastPosAux = cPosAux;
            return;
        }

        dPosLeft  = cPosLeft  - lastPosLeft;
        dPosRight = cPosRight - lastPosRight;
        dPosAux   = cPosAux  - lastPosAux;

        heading = (cPosRight-cPosLeft)/TICKS_PER_DEGREE-headingOffsetDegrees;

        dH = (dPosRight-dPosLeft)/TICKS_PER_DEGREE;

        dY = -((dPosLeft+dPosRight)/2.0);
        dX = dPosAux;

        dY /= TICKS_PER_INCH_NORMAL;
        dX /= TICKS_PER_INCH_STRAFE;

        dX -= dH * AUX_SPIN_SLIP;

        instantVector.setCartesian(dX, dY);

        trackingVector.addPolar(instantVector.getMag(), heading+instantVector.getDir());

        lastPosLeft = cPosLeft;
        lastPosRight = cPosRight;
        lastPosAux = cPosAux;

        oldX = x;
        oldY = y;

        x = trackingVector.getX();
        y = trackingVector.getY();

        if(lastUpdateTime != 0)
        {
            double deltaDisplacement = Math.sqrt(Math.pow(x-oldX, 2) + Math.pow(y-oldY, 2));
            long deltaTime = System.currentTimeMillis() - lastUpdateTime;
            double speed = deltaDisplacement / (deltaTime/1000.0);

            speedometer.add(speed);
        }

        lastUpdateTime = System.currentTimeMillis();
    }

    public void clear()
    {
        trackingVector.clear();
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public double getHeading()
    {
        return heading;
    }

    public double speed()
    {
        return speedometer.getMean();
    }

    public double getWheelHead()
    {
        return wheelHead;
    }
}