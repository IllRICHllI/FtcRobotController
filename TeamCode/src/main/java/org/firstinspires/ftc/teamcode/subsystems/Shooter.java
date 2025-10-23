package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.util.PIDFController;

public class Shooter {
    private final DcMotorEx motor;
    private final VoltageSensor vSensor;
    private final PIDFController pidf;
    private double wheelRpmRef;
    private double ticksPerSecRef;
    private final double ticksPerSecPerWheelRpm = 1120.0 / 960.0;
    private long lastNs;
    private boolean closedLoop = true;
    private double openLoopPower = 0.0;
    private double lastPower = 0.0;

    public Shooter(HardwareMap hw, String motorName, PIDFController controller) {
        motor = hw.get(DcMotorEx.class, motorName);
        vSensor = hw.voltageSensor.iterator().next();
        pidf = controller;
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lastNs = System.nanoTime();
    }

    public void setWheelRpmRef(double rpm) {
        wheelRpmRef = rpm;
        ticksPerSecRef = rpm * ticksPerSecPerWheelRpm;
        closedLoop = rpm > 0.0;
    }

    public double getWheelRpmRef() { return wheelRpmRef; }

    public double getTicksPerSec() { return motor.getVelocity(); }

    public void setOpenLoopPower(double p) {
        openLoopPower = Math.max(-1.0, Math.min(1.0, p));
        closedLoop = false;
    }

    public double getLastPower() { return lastPower; }

    public double getBattery() { return vSensor.getVoltage(); }

    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastNs) * 1e-9;
        lastNs = now;

        if (!closedLoop) {
            lastPower = openLoopPower;
            motor.setPower(lastPower);
            return;
        }

        double vBatt = vSensor.getVoltage();
        double meas = getTicksPerSec();
        double u = pidf.update(ticksPerSecRef, 0, meas, dt, vBatt);
        lastPower = u;
        motor.setPower(u);
    }

    public void stop() {
        lastPower = 0.0;
        motor.setPower(0);
        closedLoop = false;
        openLoopPower = 0.0;
    }
}
