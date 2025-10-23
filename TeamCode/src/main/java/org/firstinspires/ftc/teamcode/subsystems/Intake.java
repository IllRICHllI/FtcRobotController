package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx motor;

    public Intake(HardwareMap hw, String name) {
        motor = hw.get(DcMotorEx.class, name);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void set(double power) { motor.setPower(power); }
    public void stop() { motor.setPower(0); }
}
