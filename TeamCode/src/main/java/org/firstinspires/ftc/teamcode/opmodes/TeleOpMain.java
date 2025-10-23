package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@TeleOp(name = "TeleOpMain", group = "Main")
public class TeleOpMain extends OpMode {

    private MecanumDrive drive;
    private Shooter shooter;
    private Intake intake;
    private Feeder feeder;

    private enum ShootState { IDLE, SPOOL, FILL, PUSH, RECOVER }
    private ShootState s = ShootState.IDLE;
    private long t0;
    private double tolTicks = 150.0;
    private long holdMs = 80;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, "front_left_motor", "front_right_motor", "back_left_motor", "back_right_motor");

        PIDFController pidf = new PIDFController(
                0.0006,  // kP
                0.000,   // kI
                0.0002,  // kD
                0.10,    // kS
                0.000385,  // kV
                0.0      // kA
        );

        shooter = new Shooter(hardwareMap, "shooter_motor", pidf);
        intake = new Intake(hardwareMap, "intake_motor");
        feeder = new Feeder(hardwareMap, "ball_triger");

        // ⚙️ Shooter apagado al inicio
        shooter.setWheelRpmRef(0.0);

        telemetry.addLine("Status: Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------- CONTROL MECANUM ----------
        double vx = -gamepad1.left_stick_y;
        double vy = gamepad1.left_stick_x;

        Double omega = null;
        if (Math.abs(gamepad1.right_stick_x) > 0.05) {
            omega = Double.valueOf(gamepad1.right_stick_x);
        }

        if (omega != null) drive.resetHoldHeading();
        drive.driveField(vx, vy, omega);

        // ---------- CONTROL DE SHOOTER ----------
        shooter.update();

        boolean shootBtn = gamepad1.b;  // 🔁 ahora con el botón B
        double rpmObjetivo = 2000.0;

        switch (s) {
            case IDLE:
                shooter.setWheelRpmRef(0.0);
                intake.stop();
                feeder.stop();
                if (shootBtn) {
                    s = ShootState.SPOOL;
                    shooter.setWheelRpmRef(rpmObjetivo);
                    t0 = System.currentTimeMillis();
                }
                break;

            case SPOOL:
                if (Math.abs(shooter.getTicksPerSec() - rpmObjetivo * 1.1666667) <= tolTicks) {
                    if (System.currentTimeMillis() - t0 >= holdMs) {
                        s = ShootState.FILL;
                        t0 = System.currentTimeMillis();
                        intake.set(0.8);
                    }
                } else {
                    t0 = System.currentTimeMillis();
                }
                if (!shootBtn) s = ShootState.IDLE;
                break;

            case FILL:
                if (System.currentTimeMillis() - t0 >= 200) {
                    s = ShootState.PUSH;
                    t0 = System.currentTimeMillis();
                    feeder.push(1.0);
                }
                if (!shootBtn) {
                    intake.stop();
                    feeder.stop();
                    s = ShootState.IDLE;
                }
                break;

            case PUSH:
                if (System.currentTimeMillis() - t0 >= 150) {
                    feeder.stop();
                    s = ShootState.RECOVER;
                    t0 = System.currentTimeMillis();
                }
                if (!shootBtn) {
                    intake.stop();
                    feeder.stop();
                    s = ShootState.IDLE;
                }
                break;

            case RECOVER:
                if (Math.abs(shooter.getTicksPerSec() - rpmObjetivo * 1.1666667) <= tolTicks) {
                    if (System.currentTimeMillis() - t0 >= holdMs) {
                        s = ShootState.FILL;
                        t0 = System.currentTimeMillis();
                    }
                } else {
                    t0 = System.currentTimeMillis();
                }
                if (!shootBtn) {
                    intake.stop();
                    feeder.stop();
                    s = ShootState.IDLE;
                }
                break;
        }

        // ---------- AJUSTE DE RPM (OPCIONAL) ----------
        if (gamepad1.dpad_up) shooter.setWheelRpmRef(2200.0);
        if (gamepad1.dpad_down) shooter.setWheelRpmRef(1800.0);

        // ---------- TELEMETRÍA ----------
        telemetry.addData("RPM Ref", shooter.getWheelRpmRef());
        telemetry.addData("Ticks/s", shooter.getTicksPerSec());
        telemetry.addData("Shooter State", s);
        telemetry.addData("Heading (rad)", drive.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        shooter.stop();
        intake.stop();
        feeder.stop();
    }
}
