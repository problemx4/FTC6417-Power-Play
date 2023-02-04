package org.firstinspires.ftc.teamcode.BluCru6417;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalDetectorPipeline extends OpenCvPipeline{
    int subMatRectX;
    int subMatRectY;
    int subMatRectWidth;
    int subMatRectHeight;

    Rect subMatRect;

    public int position = 1;
    public double redTotal;
    public double greenTotal;
    public double blueTotal;

    Mat copy = new Mat();

    public SignalDetectorPipeline(double centerx, double centery, int width, int height, int camWidth, int camHeight){
        subMatRectX = (int)(camWidth * centerx) - (width / 2);
        subMatRectY = (int)(camHeight * centery) - (height / 2);
        subMatRectWidth = width;
        subMatRectHeight = height;

        subMatRect = new Rect(subMatRectX, subMatRectY, subMatRectWidth, subMatRectHeight);
    }

    public Mat processFrame(Mat input){
        input.copyTo(copy);

        if(copy.empty()){
            return input;
        }
        Imgproc.rectangle(copy, subMatRect, new Scalar(0,255,0));
        Mat subMat = copy.submat(subMatRect);

        //rgb color mode:
        //val[] = {red,green,blue}
        redTotal = Core.sumElems(subMat).val[0];
        greenTotal = Core.sumElems(subMat).val[1];
        blueTotal = Core.sumElems(subMat).val[2];

        double max = Math.max(redTotal, Math.max(greenTotal, blueTotal));

        if(max == redTotal){
            position = 0;
        }
        if(max == greenTotal){
            position = 1;
        }
        if(max == blueTotal){
            position = 2;
        }

        subMat.release();
        //copy.release();
        return copy;
    }
}
