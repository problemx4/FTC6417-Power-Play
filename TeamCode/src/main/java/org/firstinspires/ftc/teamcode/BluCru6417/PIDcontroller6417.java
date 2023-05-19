package org.firstinspires.ftc.teamcode.BluCru6417;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontroller6417 {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public double target = 0;
    public double lastTarget = target;

    public double a = 0.999; // a can be anything from 0 < a < 1
    public double previousFilterEstimate = 0;
    public double currentFilterEstimate = 0;

    public double errorTolerance = 0;
    public double derivativeTolerance = 0;

    public double error;
    public double integralSum;
    public double derivative;

    public double lastError;

    public ElapsedTime elapsedTime;

    public PIDcontroller6417(double P, double I, double D){
        setCoefficients(P,I,D);
        elapsedTime = new ElapsedTime();
    }

    public void setCoefficients(double P, double I, double D){
        kP = P;
        kI = I;
        kD = D;
    }

    public void setTarget(double tar){
        target = tar;
    }

    public double getTarget(){
        return target;
    }

    public void setTolerances(double errorT, double derivativeT){
        errorTolerance = errorT;
        derivativeTolerance = derivativeT;
    }

    public void resetTimer(){
        elapsedTime.reset();
    }

    public void resetIntegral(){
        integralSum = 0;
    }

    public boolean isBusy(){
        return Math.abs(error) > errorTolerance || Math.abs(derivative) > derivativeTolerance;
    }

    public double calculate(double current){
        //get the time since last call
        double timeDelta = elapsedTime.seconds();

        //calculate error
        error = target - current;

        //derivative with low pass filter
        // filter out high frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * (error - lastError);
        previousFilterEstimate = currentFilterEstimate;

        //calculate derivative
        derivative = currentFilterEstimate / timeDelta;

        //calculate integral
        integralSum += error * timeDelta;

        if(lastTarget != target){
            resetIntegral();
        }

        lastError = error;
        lastTarget = target;

        resetTimer();

        if(!isBusy()){
            resetIntegral();
            return 0.0;
        }
        return (error * kP) + (integralSum * kI) + (derivative * kD);
    }
}
