package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    private final CRServo servo;

    public Feeder(HardwareMap hw, String name) {
        servo = hw.get(CRServo.class, name);
    }

    public void push(double power) { servo.setPower(power); }
    public void stop() { servo.setPower(0); }
}
