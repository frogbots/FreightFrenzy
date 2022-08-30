package ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//import net.frogbots.skystone.opmodes.auto.AbortAutoNowException;

public class FrogBNO055
{
    private BNO055IMU imu;
    private static float yaw, pitch, roll;
    private int numAttempts = 0;

    public FrogBNO055(BNO055IMU imu)
    {
        this.imu = imu;
    }

    public float pollAndGetYaw()
    {
        poll();
        return yaw;
    }

    public float pollAndGetPitch()
    {
        poll();
        return pitch;
    }

    public float pollAndGetRoll()
    {
        poll();
        return roll;
    }

    public static float getYaw()
    {
        return yaw;
    }

    public float getPitch()
    {
        return pitch;
    }

    public float getRoll()
    {
        return roll;
    }


    public void poll()
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yaw = orientation.firstAngle;
        pitch = orientation.secondAngle;
        roll = orientation.thirdAngle;

        if(yaw == 0.0 && pitch == 0.0 && roll == 0.0)
        {
            if(numAttempts > 3)
            {
   //             throw new RuntimeException("IMU returned garbage data after 3 attempts, killing program");
                //Globals.CRITICAL_ERR_ABORT_NOW = true;
                //Globals.requestOpModeStop();
 //               throw new AbortAutoNowException("IMU returned garbage data after 3 attempts, killing program");
            }
            else
            {
                numAttempts++;
                System.out.println("GOT GARBAGE DATA FROM IMU");
                poll();
            }
        }
        else
        {
            numAttempts = 0;
        }
    }

    public void init()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.useExternalCrystal   = true;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.mode                 = BNO055IMU.SensorMode.IMU;

        imu.initialize(parameters);
    }

    public String getCalibrationStatus()
    {
        return imu.getCalibrationStatus().toString();
    }
}