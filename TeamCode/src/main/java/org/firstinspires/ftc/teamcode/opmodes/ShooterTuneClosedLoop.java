package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Autonomous(name="ShooterTuneClosedLoop", group="Tune")
public class ShooterTuneClosedLoop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double kP = 0.0006;
        double kI = 0.0;
        double kD = 0.0002;
        double kS = 0.10;
        double kV = 0.000385;
        double kA = 0.0;

        PIDFController pidf = new PIDFController(kP,kI,kD,kS,kV,kA);
        Shooter shooter = new Shooter(hardwareMap, "shooter_motor", pidf);

        double rpm = 2000.0;
        boolean enabled = false;

        telemetry.addLine("A: on/off | Up/Down: rpm +/-100 | Right/Left: kP x1.10/x0.9 0 | Y/X: kD x1.20/x0.83 | B: reset");
        telemetry.update();
        waitForStart();

        long tStep = System.currentTimeMillis();
        while (opModeIsActive()) {
            if (gamepad1.a) { enabled = !enabled; sleep(160); }
            if (gamepad1.b) { pidf.reset(); sleep(160); }
            if (gamepad1.dpad_up) { rpm += 100.0; sleep(140); }
            if (gamepad1.dpad_down) { rpm = Math.max(0.0, rpm - 100.0); sleep(140); }

            if (gamepad1.dpad_right) {
                kP *= 1.10;
                pidf = new PIDFController(kP,kI,kD,kS,kV,kA);
                shooter = new Shooter(hardwareMap, "shooter", pidf);
                sleep(160);
            }
            if (gamepad1.dpad_left) {
                kP *= 0.90;
                pidf = new PIDFController(kP,kI,kD,kS,kV,kA);
                shooter = new Shooter(hardwareMap, "shooter", pidf);
                sleep(160);
            }
            if (gamepad1.y) {
                kD *= 1.20;
                pidf = new PIDFController(kP,kI,kD,kS,kV,kA);
                shooter = new Shooter(hardwareMap, "shooter", pidf);
                sleep(160);
            }
            if (gamepad1.x) {
                kD *= 0.83;
                pidf = new PIDFController(kP,kI,kD,kS,kV,kA);
                shooter = new Shooter(hardwareMap, "shooter", pidf);
                sleep(160);
            }

            if (enabled) shooter.setWheelRpmRef(rpm);
            else shooter.setWheelRpmRef(0.0);

            shooter.update();

            double refTps = rpm * (1120.0 / 960.0);
            double err = refTps - shooter.getTicksPerSec();

            telemetry.addData("on", enabled);
            telemetry.addData("rpm_ref", rpm);
            telemetry.addData("ticks_ref", refTps);
            telemetry.addData("ticks_now", shooter.getTicksPerSec());
            telemetry.addData("err_tps", err);
            telemetry.addData("kP", kP);
            telemetry.addData("kD", kD);
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addData("power_out", shooter.getLastPower());
            telemetry.addData("vBatt", shooter.getBattery());
            telemetry.update();

            idle();
        }

        shooter.stop();
    }
}
