/*package ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UltimateGoal.RingDetectionRed;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import drivers.MaxSonarI2CXL;



//@TeleOp
public class SetupSensors extends LinearOpMode
{
    OpenCvCamera phoneCam;

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

    @Override
    public void runOpMode() {
        RightSonar = hardwareMap.get(MaxSonarI2CXL.class, "RightSonar");
        BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");
        LeftSonar = hardwareMap.get(MaxSonarI2CXL.class, "LeftSonar");
        //BackSonar = hardwareMap.get(MaxSonarI2CXL.class, "BackSonar");

        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive()) {
            RcmToInch = RightSonar.getDistanceSync();
            BcmToInch = BackSonar.getDistanceSync();
            LcmToInch = LeftSonar.getDistanceSync();
            Rinch = RcmToInch / 2.54;
            Linch = LcmToInch / 2.54;
            Binch = BcmToInch / 2.54;
            telemetry.addData("Right Dist inch", Rinch + 1);
            telemetry.addData("Back Dist inch", Binch + 1);
            telemetry.addData("Left Dist inch", Linch + 2);
            telemetry.update();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);


            RingDetectionRed RingPipline = new RingDetectionRed();

            phoneCam.setPipeline(RingPipline);


            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }
            });

            telemetry.addLine("Waiting for start");
            telemetry.update();


            waitForStart();

            while (opModeIsActive()) {


                telemetry.addData("Frame Count", phoneCam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
                telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
                telemetry.addData("Density", RingPipline.GetLastDensity());
                telemetry.addData("Control", RingPipline.GetLastControlDensity());
                telemetry.addData("Position", RingPipline.GetPosition());
                telemetry.addData("Position", RingPipline.GetLastRatioDensity());
                telemetry.addData("Right Dist inch", Rinch + 1);
                telemetry.addData("Back Dist inch", Binch + 1);
                telemetry.addData("Left Dist inch", Linch + 1);
                telemetry.update();

                if (gamepad1.a) {

                    phoneCam.stopStreaming();
                    //phoneCam.closeCameraDevice();
                }


                sleep(100);
            }
        }


        class SamplePipeline extends OpenCvPipeline {
            boolean viewportPaused = false;
            Mat YCRCB = new Mat();
            Mat CR = new Mat();


            @Override
            public Mat processFrame(Mat input) {
                Imgproc.cvtColor(input, YCRCB, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(YCRCB, CR, 1);




                return CR;
            }

            @Override
            public void onViewportTapped() {


                viewportPaused = !viewportPaused;

                if (viewportPaused) {
                    phoneCam.pauseViewport();
                } else {
                    phoneCam.resumeViewport();
                }
            }
        }
    }
}



 */