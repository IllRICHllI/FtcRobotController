package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    private double integralSum = 0;
    private double lastError = 0;
    private double maxIntSum;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd, double Kf, double maxIntegralSum) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.maxIntSum = maxIntegralSum;
        timer.reset();
    }

    public double calculate(double reference, double state) {
        double error = reference - state;

        double dt = timer.seconds();
        if (dt < 1e-5) dt = 1e-5;

        integralSum += error * dt;
        if (integralSum > maxIntSum) integralSum = maxIntSum;
        if (integralSum < -maxIntSum) integralSum = -maxIntSum;

        double derivative = (error - lastError) / dt;

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * reference);

        lastError = error;
        timer.reset();

        return output;
    }

    public void resetIntegralSum() {
        integralSum = 0;
    }
}
