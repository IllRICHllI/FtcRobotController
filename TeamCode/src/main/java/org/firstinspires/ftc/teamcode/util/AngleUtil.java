package org.firstinspires.ftc.teamcode.util;

public class AngleUtil {
    public static double norm(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
