package org.firstinspires.ftc.teamcode.BluCru6417;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.BluCru6417.PIDcontroller6417;

public class PIDslider6417 extends PIDcontroller6417 {
    double kG;
    double power;

    public PIDslider6417(double P, double I, double D, double F){
        super(P,I,D);
        kG = F;
        power = 1;
    }

    public void setCoefficients(double P, double I, double D, double F){
        super.setCoefficients(P,I,D);
        kG = F;
    }

    @Override
    public void setTarget(double tar){
        target = Range.clip(tar / 145.1, ControlConstants.sliderMinPos, ControlConstants.sliderMaxPos);
    }

    public void setPower(double pow){
        power = pow;
    }

    @Override
    public double calculate(double current){
        double revs = current / 145.1;

        double output = super.calculate(revs);

        return Range.clip(output + kG, -Math.abs(power), Math.abs(power));
    }
}
