package ftc.teamcode.FreightFrenzy;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import ftc.teamcode.DropPositions;

public class TSEDetectionRed extends OpenCvPipeline
{
    double TSEDenisty = 125;
    //double ThreeRingThreshold = 1.16;
    double Density;
    double Density2;
    double ControlDensity;
    double RatioDensity;
    double RatioDensity2;
    DropPositions Position;
    int width = 35;
    int hight = 40;
    Mat CutoutMat;
    Mat CutoutMat2;
    Mat CutoutMat3;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    Point TopLeftPoint = new Point(155,23); // (0,0) is top left // 47 158
    Point BottomRightPoint = new Point(TopLeftPoint.x + width, TopLeftPoint.y + hight); // reminder to adjust the X val off of ultra sonic distance

    Point TopLeftPoint2 = new Point(155,175); // (0,0) is top left
    Point BottomRightPoint2 = new Point(TopLeftPoint2.x + width, TopLeftPoint2.y + hight);

    Point TopLeftPoint3 = new Point(150,135); // (0,0) is top left // 47 158
    Point BottomRightPoint3 = new Point(TopLeftPoint3.x + 20, TopLeftPoint3.y + 10);


    @Override
    public void init (Mat input) {
        Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb,Cr,1);
        CutoutMat = Cr.submat(new Rect(TopLeftPoint, BottomRightPoint));
        CutoutMat2 = Cr.submat(new Rect(TopLeftPoint2, BottomRightPoint2));
        CutoutMat3 = Cr.submat(new Rect(TopLeftPoint3, BottomRightPoint3));

    }
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb,Cr,1);
        Imgproc.rectangle(
                input,
                TopLeftPoint,
                BottomRightPoint,
                new Scalar(0, 0, 255), 2);
        Density = (int) Core.mean(CutoutMat).val[0];
        Imgproc.rectangle(
                input,
                TopLeftPoint2,
                BottomRightPoint2,
                new Scalar(0, 0, 255), 2);
        Density2 = (int) Core.mean(CutoutMat2).val[0];
        Imgproc.rectangle(
                input,
                TopLeftPoint3,
                BottomRightPoint3,
                new Scalar(0, 0, 255), 2);
        ControlDensity = (int) Core.mean(CutoutMat3).val[0];
        return input;
    }
    public double GetLastDensity() {
        return Density;
    }
    public double GetLastDensity2() {
        return Density2;
    }
    public double GetLastControlDensity() {
        return ControlDensity;
    }
    public double GetLastRatioDensity() {
        return RatioDensity;
    }
    public double GetLastRatioDensity2() {
        return RatioDensity2;
    }
    public DropPositions GetPosition() {
        RatioDensity = Density/ControlDensity;
        RatioDensity2 = Density2/ControlDensity;
        if (Density < TSEDenisty) {
           Position = DropPositions.B;
        }
        if (Density2 < TSEDenisty) {
            Position = DropPositions.C;
        }
        else if (Density > TSEDenisty && Density2 > TSEDenisty){
            Position = DropPositions.A;
        }
        return Position;
    }
}