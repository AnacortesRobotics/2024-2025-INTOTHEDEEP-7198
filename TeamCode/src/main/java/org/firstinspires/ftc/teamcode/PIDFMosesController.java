package org.firstinspires.ftc.teamcode;

public class PIDFMosesController {

    private double Pval;
    private double Dval;
    private double Ival;
    private boolean invert;
    private double lastError = 0;
    private double lastTime = 0;
    private double Itotal;

    public void init(double P, double I, double D, boolean invert) {
        Pval = P;
        Ival = I;
        Dval = D;
        this.invert = invert;
    }

    public double update(double target, double current, double maxSpeed) {
        double error = target - current;
        double timeChange = System.currentTimeMillis() / 1000.0 - lastTime;
        Itotal += Ival * error / timeChange;
        double output = Pval * error +
                Dval * (error - lastError) / timeChange +
                Itotal;
        output = Math.min(output, maxSpeed);
        output = Math.max(output, -maxSpeed);
        lastError = error;
        lastTime = System.currentTimeMillis() / 1000.0;
        return invert ? output : -output;
    }

}
