package org.firstinspires.ftc.teamcode.util;

public class PIDFController {
    private double kP, kI, kD, kS, kV, kA;
    private double integral, prevError;
    private boolean first = true;

    public PIDFController(double kP, double kI, double kD, double kS, double kV, double kA) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        this.kS = kS; this.kV = kV; this.kA = kA;
    }

    public void reset() { integral = 0; first = true; }

    public double update(double ref, double refDot, double meas, double dt, double vBatt) {
        double error = ref - meas;
        integral += error * dt;
        double deriv = first ? 0 : (error - prevError) / dt;
        prevError = error; first = false;
        double ff = (kS * Math.signum(ref)) + (kV * ref) + (kA * refDot);
        if (vBatt > 0) ff *= (12.0 / vBatt);
        double u = ff + (kP * error) + (kI * integral) + (kD * deriv);
        if (u > 1) u = 1;
        if (u < -1) u = -1;
        return u;
    }
}
