package org.firstinspires.ftc.teamcode.BluCru6417;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontroller6417 {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public double target = 0;

    public double errorTolerance = 5;
    public double derivativeTolerance = 5;

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
        elapsedTime.reset();
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
        return !(error < errorTolerance && derivative < derivativeTolerance);
    }

    public double calculate(double current){
        if(!isBusy()){
            return 0.0;
        }

        //get the time since last call
        double timeDelta = elapsedTime.seconds();

        //calculate error
        error = target - current;

        //calculate derivative
        derivative = (error - lastError) / timeDelta;

        //calculate integral
        integralSum += error * timeDelta;

        lastError = error;

        elapsedTime.reset();

        return (error * kP) + (integralSum * kI) + (derivative * kD);
    }
}
