package ftc.teamcode;

public class AngleManipulator
{
    public static double manipulateDiscontinuity(double targetAngle, double actualAngle)
    {
        double transformedAngle;
        if(targetAngle > actualAngle || targetAngle < actualAngle)
        {
            transformedAngle = actualAngle - targetAngle;
        }
        else
        {
            transformedAngle = 0;
        }

        if(Math.abs(transformedAngle) > 180)
        {
            if (transformedAngle > 180)
            {
                transformedAngle -= 360;
            }
            else if (transformedAngle < 180)
            {
                transformedAngle += 360;
            }
        }

        return transformedAngle * -1;
    }
}