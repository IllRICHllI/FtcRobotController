package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Autonomous(name="ShooterCharOpenLoop", group="Tune")
public class ShooterCharOpenLoop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PIDFController dummy = new PIDFController(0,0,0,0,0,0);
        Shooter shooter = new Shooter(hardwareMap, "shooter_motor", dummy);
        double power = 0.20;
        boolean enabled = false;

        telemetry.addLine("A: enable/disable  |  dpad_up/down: power +/-0.05  |  X: stop");
        telemetry.update();
        waitForStart();

        long settleMs = 1000;
        long tSettled = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                shooter.stop();
                enabled = false;
            }
            if (gamepad1.a) {
                enabled = !enabled;
                tSettled = System.currentTimeMillis();
            }
            if (gamepad1.dpad_up) {
                power = Math.min(1.0, power + 0.05);
                tSettled = System.currentTimeMillis();
                sleep(180);
            }
            if (gamepad1.dpad_down) {
                power = Math.max(0.0, power - 0.05);
                tSettled = System.currentTimeMillis();
                sleep(180);
            }

            if (enabled) shooter.setOpenLoopPower(power);
            else shooter.setOpenLoopPower(0.0);

            shooter.update();

            boolean ready = System.currentTimeMillis() - tSettled > settleMs;

            telemetry.addData("enabled", enabled);
            telemetry.addData("power_cmd", power);
            telemetry.addData("vBatt", shooter.getBattery());
            telemetry.addData("ticks_per_sec", shooter.getTicksPerSec());
            telemetry.addData("ready_to_sample", ready);
            telemetry.update();

            idle();
        }

        shooter.stop();
    }
}
