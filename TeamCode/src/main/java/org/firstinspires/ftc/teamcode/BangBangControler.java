package org.firstinspires.ftc.teamcode;

public class BangBangControler {

    private double allowedError = 0;
    private boolean invert = false;

    public void init(double allowedError, boolean invert) {
        this.allowedError = allowedError;
        this.invert = invert;
    }

    public double update(double target, double current, double speed) {
        if (Math.abs(target - current) <= allowedError) {
            return 0;
        }
        if (target >= current) {
            return invert ? speed : -speed;
        } else {
            return invert ? -speed : speed;
        }
    }


}
