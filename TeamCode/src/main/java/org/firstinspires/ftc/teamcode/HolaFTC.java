package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hola FTC", group = "Pruebas")
public class HolaFTC extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Estado", "Esperando inicio...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Hola", "¡Tu robot está vivo!");
            telemetry.update();
        }
    }
}
