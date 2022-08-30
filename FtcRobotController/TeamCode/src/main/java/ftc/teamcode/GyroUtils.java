package ftc.teamcode;

import org.firstinspires.ftc.teamcode.AccerlationControlledDrivetrainPowerGeneratorForAuto;
import org.firstinspires.ftc.teamcode.NotFF.FrogTeleOp;
import org.firstinspires.ftc.teamcode.Globals;
//import org.firstinspires.ftc.teamcode.UltimateGoal.frogTeleOp;

import org.firstinspires.ftc.teamcode.control.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.DriveTrain;
import org.firstinspires.ftc.teamcode.robotComponents.drivebase.MotorPowers;


public class GyroUtils
{
    private static FrogBNO055 imu;
    private DriveTrain driveTrain;

    public GyroUtils(FrogBNO055 imu, DriveTrain driveTrain)
    {
        this.imu = imu;
        this.driveTrain = driveTrain;
    }

    private static float getHeading()
    {
        return 0; //FrogTeleOp.getYaw();
    }

    //------------------------------------------------------------------------------------------------
    // Rotate
    //------------------------------------------------------------------------------------------------

    public double gyroRotate(double target, double Kp)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());
        Globals.robot.setMotorPowers(-err*Kp, err*Kp);
        return err;
    }

    public static double gyroRotate(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double target, double Kp)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        double turnPower = err*Kp;

        Globals.robot.setMotorPowers(acclCtrl.getAccelerationControlledPowers(new MotorPowers(-turnPower, turnPower)));

        return err;
    }

    public double gyroRotate(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double target, double Kp, double minPow)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        double turnPower = err*Kp;

        turnPower = enforceMinPower(turnPower, minPow);

        driveTrain.setMotorPowers(acclCtrl.getAccelerationControlledPowers(new MotorPowers(-turnPower, turnPower)));

        return err;
    }

    //------------------------------------------------------------------------------------------------
    // Forward / backward
    //------------------------------------------------------------------------------------------------

    public double gyroStraight(double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(MecanumDrive.calcCartesian(pow, 0, 0), target, Kp);
    }

    public double gyroStraight(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(acclCtrl, MecanumDrive.calcCartesian(pow, 0, 0), target, Kp);
    }

    //------------------------------------------------------------------------------------------------
    // Strafing
    //------------------------------------------------------------------------------------------------

    public double gyroStrafe(double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(MecanumDrive.calcCartesian(0, pow, 0), target, Kp);
    }

    public double gyroStrafe(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, double pow, double target, double Kp)
    {
        return setPowersWithHeadingCorrection(acclCtrl, MecanumDrive.calcCartesian(0, pow, 0), target, Kp);
    }

    //------------------------------------------------------------------------------------------------
    // Misc
    //------------------------------------------------------------------------------------------------
    public double setPowersWithHeadingCorrection(MotorPowers powers, double target, double Kp)
    {
        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        System.out.println("Gyro err " + err);

        powers.frontLeft  -= err*Kp;
        powers.rearLeft   -= err*Kp;
        powers.frontRight += err*Kp;
        powers.rearRight  += err*Kp;

        driveTrain.setMotorPowers(powers);

        return err;
    }

    public double setPowersWithHeadingCorrection(AccerlationControlledDrivetrainPowerGeneratorForAuto acclCtrl, MotorPowers powers, double target, double Kp)
    {
        powers = acclCtrl.getAccelerationControlledPowers(powers);

        double err = AngleManipulator.manipulateDiscontinuity(target, getHeading());

        System.out.println("setPowersWithHeadingCorrection gyro err " + err);

        powers.frontLeft  -= err*Kp;
        powers.rearLeft   -= err*Kp;
        powers.frontRight += err*Kp;
        powers.rearRight  += err*Kp;

        driveTrain.setMotorPowers(powers);

        //System.out.println("setPowersWithHeadingCorrection prenormalized powers: " + powers.toString());

        return err;
    }

    public static double enforceMinPower(double input, double minPower)
    {
        minPower = Math.abs(minPower); //make sure minPower is always positive

        if(input < 0) //Input power is negative
        {
            /*
             * Return the smallest (err, biggest negative) of either the user's input power, or minPower
             */
            input = Math.min(input, -minPower);
        }
        else // Input power is positive
        {
            /*
             * Return the biggest of either the user's input power, or minPower
             */
            input = Math.max(input, minPower);
        }

        return input;
    }
}
