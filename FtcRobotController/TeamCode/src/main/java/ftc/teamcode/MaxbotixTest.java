package ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivers.MaxSonarI2CXL;

@TeleOp
public class MaxbotixTest extends LinearOpMode
{
    MaxSonarI2CXL RightSonar;
    MaxSonarI2CXL BackSonar;
    MaxSonarI2CXL LeftSonar;
    MaxSonarI2CXL FrontSonar;
    double RcmToInch = 0;
    double Rinch = 0;
    double LcmToInch = 0;
    double Linch = 0;
    double BcmToInch = 0;
    double Binch = 0;
    double FcmToInch = 0;
    double Finch = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
        //BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        FrontSonar = hardwareMap.get(MaxSonarI2CXL.class, "FrontDistance");
        //BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");

        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive())
        {
//            RcmToInch = RightSonar.getDistanceSync();
//            BcmToInch = BackSonar.getDistanceSync();
//            LcmToInch = LeftSonar.getDistanceSync();
//            Rinch = RcmToInch/2.54;
//            Linch = LcmToInch/2.54;
//            Binch = BcmToInch/2.54;
//            telemetry.addData("Right Dist inch", Rinch+1);
//            telemetry.addData("Back Dist inch", Binch+1);
//            telemetry.addData("Left Dist inch", Linch+1);
//            telemetry.update();
            FcmToInch = FrontSonar.getDistanceSync();
            Finch = RcmToInch/2.54;
            telemetry.addData("Front Dist inch", Finch+1);
            telemetry.update();
        }
    }
}
