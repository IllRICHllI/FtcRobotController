package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Combined TeleOp")
public class CombinedTeleOp extends OpMode {

    private MecanumDrive drive;
    private Shooter shooter;

    private double forward, strafe, rotate;

    // 💡 Ahora puedes cambiar libremente las RPM deseadas
    private final double SHOOTER_RPM_HIGH = 100.0;
    private final double SHOOTER_RPM_OFF = 0.0;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, "front_left_motor", "back_left_motor", "front_right_motor", "back_right_motor", "imu");
        shooter = new Shooter(hardwareMap, "shooter_motor");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        if (gamepad1.options) {
            drive.resetFieldHeading();
            telemetry.addData("Field Centric", "Heading Reset!");
        }

        drive.driveFieldRelative(forward, strafe, rotate);

        if (gamepad2.a) {
            shooter.setTargetVelocityRPM(SHOOTER_RPM_HIGH);
        } else if (gamepad2.b) {
            shooter.setTargetVelocityRPM(SHOOTER_RPM_OFF);
        }

        shooter.update();

        telemetry.addData("Drivetrain Heading (Rad)", drive.getRobotHeading(AngleUnit.RADIANS));
        telemetry.addData("Shooter Target Velocity (ticks/s)", shooter.getTargetVelocity());
        telemetry.addData("Shooter Current Velocity (ticks/s)", shooter.getCurrentVelocity());
        telemetry.addData("Shooter Target RPM", shooter.getTargetRPM());
        telemetry.addData("Shooter Current RPM", shooter.getCurrentRPM());

        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
