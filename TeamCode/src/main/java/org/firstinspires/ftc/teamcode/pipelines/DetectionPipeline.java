package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Moments;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;
@Config
public class DetectionPipeline extends OpenCvPipeline
{
    //  private int width;
    Boolean isFacing;
    double distance;
    private Telemetry telemetry;
    private Boolean two;
    public DetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public static int l1 = 66;
    public static int l2 = 41;
    public static int l3 = 108;

    public static int u1 = 216;
    public static int u2 = 121;
    public static int u3 = 148;
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2Lab);
        //  Rect crop = new Rect(0, input.height()/2, input.width(), input.height()/2);
        telemetry.addData("Width", input.width());
        //  mat = new Mat(mat, crop);
        Scalar lower = new Scalar(l1, l2, l3);
        Scalar upper = new Scalar(u1, u2, u3);

        Mat thresh = new Mat();

        Core.inRange(mat, lower, upper, thresh);
        List<MatOfPoint> contours = new ArrayList();

        Mat heirarchy = new Mat();
        Imgproc.findContours(thresh, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        double maxVal = 0;
        int maxValIdx = 0;
        if (contours.size() == 0) {
            return mat;
        }

        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        Moments m = Imgproc.moments(contours.get(maxValIdx));

        double center = m.m10/m.m00;
        telemetry.addData("Center", center);
        int width = input.width();
        isFacing = Math.abs(width/2 - center) < 25;
        distance = (width/2 - center)/Math.abs(width/2 - center);
        telemetry.addData("IsFacing", isFacing);
        telemetry.addData("Distance", distance);
        telemetry.update();
        Imgproc.circle(mat, new Point(m.m10/m.m00, m.m01/m.m00), 5, new Scalar(255, 0, 0), 40);
        Imgproc.putText(
                mat,
                isFacing.toString(),
                new Point(30, 40),
                Imgproc.FONT_HERSHEY_PLAIN,
                2,
                new Scalar(255, 255, 255),
                2
        );


        return thresh;

    }


    public Boolean isFacing() {
        return isFacing;
    }

    public double getMultiplier() {
        return distance;
    }

}