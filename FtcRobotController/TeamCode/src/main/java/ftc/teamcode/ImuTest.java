package ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ImuTest extends LinearOpMode
{
    static FrogBNO055 imu;
    public static double Yaw;

    @Override
    public void runOpMode()
    {
        imu = new FrogBNO055( hardwareMap.get(BNO055IMU.class, "external_IMU"));

        imu.init();

        waitForStart();

        telemetry.setMsTransmissionInterval(20);

        while (opModeIsActive())
        {
            imu.poll();
            telemetry.addData("Yaw", imu.getYaw());
            telemetry.addData("Pitch", imu.getPitch());
            telemetry.addData("Roll", imu.getRoll());
            telemetry.update();
        }

    }
    public static double GetYaw() {
        Yaw = imu.getYaw();
        return Yaw;
    }
}
