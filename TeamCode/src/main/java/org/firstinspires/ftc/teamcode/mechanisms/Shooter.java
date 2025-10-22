package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.controllers.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx shooterMotor;
    private PIDController pidController;

    public static final double Kp = 0.001;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
    public static final double Kf = 0.0;
    public static final double maxIntSum = 1000.0;

    private double targetVelocity = 0.0;

    // ✅ Datos correctos del REV HD Hex 40:1
    public static final double TICKS_PER_REV = 1120.0;

    public Shooter(HardwareMap hardwareMap, String motorName) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pidController = new PIDController(Kp, Ki, Kd, Kf, maxIntSum);
    }

    // Conversión de RPM → ticks/segundo
    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    // Permite establecer la velocidad deseada en RPM
    public void setTargetVelocityRPM(double rpm) {
        targetVelocity = rpmToTicksPerSecond(rpm);
        pidController.resetIntegralSum();
    }

    // También puedes usar ticks/seg directamente
    public void setTargetVelocity(double ticksPerSecond) {
        targetVelocity = ticksPerSecond;
        pidController.resetIntegralSum();
    }

    public void update() {
        double currentVelocity = shooterMotor.getVelocity();
        double power = pidController.calculate(targetVelocity, currentVelocity);
        if (power > 1.0) power = 1.0;
        if (power < -1.0) power = -1.0;
        shooterMotor.setPower(power);
    }

    public void stop() {
        targetVelocity = 0;
        shooterMotor.setPower(0);
        pidController.resetIntegralSum();
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentRPM() {
        return (shooterMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return (getTargetVelocity() * 60.0) / TICKS_PER_REV;
    }
}
