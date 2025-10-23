package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Autonomous(name="AutoMain")
public class AutoMain extends LinearOpMode {
    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, "fl","fr","bl","br");
        PIDFController pidf = new PIDFController(0.0006,0.0,0.0002,0.02,0.0003,0.0);
        Shooter shooter = new Shooter(hardwareMap, "shooter", pidf);
        Intake intake = new Intake(hardwareMap, "intake");
        Feeder feeder = new Feeder(hardwareMap, "feeder");

        shooter.setWheelRpmRef(2000.0);
        waitForStart();
        if (isStopRequested()) return;

        long tStart = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - tStart < 1500) {
            drive.driveField(0.6, 0.0, null);
            shooter.update();
        }
        drive.stop();

        double targetTurnTime = 800;
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < targetTurnTime) {
            drive.driveField(0.0, 0.0, 0.8);
            shooter.update();
        }
        drive.stop();

        long hold = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - hold < 3000) {
            shooter.update();
            if (Math.abs(shooter.getTicksPerSec() - 2000.0*1.1666667) < 150) {
                intake.set(0.7);
                sleep(200);
                feeder.push(1.0);
                sleep(160);
                feeder.stop();
                intake.stop();
                break;
            }
        }

        shooter.stop();
        drive.stop();
    }
}
