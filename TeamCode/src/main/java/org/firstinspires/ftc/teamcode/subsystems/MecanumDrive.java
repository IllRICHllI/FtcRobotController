package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrive {
    private final DcMotorEx fl, fr, bl, br;
    private final IMU imu;
    private double holdHeading;
    private double kPh = 2.0, kDh = 0.1;
    private double prevErr = 0;
    private long lastNs;

    public MecanumDrive(HardwareMap hw, String fln, String frn, String bln, String brn) {
        fl = hw.get(DcMotorEx.class, fln);
        fr = hw.get(DcMotorEx.class, frn);
        bl = hw.get(DcMotorEx.class, bln);
        br = hw.get(DcMotorEx.class, brn);

        // ⚙️ Configuración de dirección de giro
        // Normalmente los motores del lado derecho deben estar invertidos
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        // Inicializa el IMU del Control Hub
        imu = hw.get(IMU.class, "imu");

        // Orientación física del Control Hub (ajústala según tu montaje real)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        lastNs = System.nanoTime();
        holdHeading = getHeading();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public void resetHoldHeading() {
        holdHeading = getHeading();
        prevErr = 0;
        lastNs = System.nanoTime();
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void driveField(double vx, double vy, Double omegaCmd) {
        double theta = getHeading();
        double xr = vx * Math.cos(theta) + vy * Math.sin(theta);
        double yr = -vx * Math.sin(theta) + vy * Math.cos(theta);

        long now = System.nanoTime();
        double dt = (now - lastNs) * 1e-9;
        lastNs = now;

        double omega;
        if (omegaCmd == null) {
            double err = holdHeading - theta;
            err = Math.atan2(Math.sin(err), Math.cos(err));
            double der = (err - prevErr) / Math.max(1e-3, dt);
            prevErr = err;
            omega = kPh * err + kDh * der;
        } else {
            holdHeading = theta;
            omega = omegaCmd;
        }

        double flp = xr + yr + omega;
        double frp = xr - yr - omega;
        double blp = xr - yr + omega;
        double brp = xr + yr - omega;

        double max = Math.max(1.0, Math.max(Math.abs(flp), Math.abs(frp)));
        max = Math.max(max, Math.max(Math.abs(blp), Math.abs(brp)));

        fl.setPower(flp / max);
        fr.setPower(frp / max);
        bl.setPower(blp / max);
        br.setPower(brp / max);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
