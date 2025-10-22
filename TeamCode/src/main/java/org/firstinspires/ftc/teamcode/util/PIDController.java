package org.firstinspires.ftc.teamcode.util; // Paquete recomendado para utilidades

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry; // Incluir Telemetry para depuración (opcional, pero útil)

/**
 * Controlador Proporcional-Integral-Derivativo (PID) con Feedforward.
 * Optimizado para control de VELOCIDAD (Ticks por segundo).
 */
public class PIDController {

    // Coeficientes
    private double Kp, Ki, Kd, Kf;

    // Variables de estado persistentes
    public double integralSum = 0; // Debe ser público para resetearlo al detener el motor [8]
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Anti-Windup Limit [5]
    private final double INTEGRAL_SUM_LIMIT = 0.25; 

    /**
     * Constructor del controlador PID.
     */
    public PIDController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        timer.reset();
    }

    /**
     * Calcula la potencia de salida para alcanzar la velocidad de referencia.
     * @param reference Velocidad objetivo (e.g., Ticks/Segundo).
     * @param state Velocidad actual del motor (en las mismas unidades).
     * @return La potencia del motor ajustada (entre -1.0 y 1.0).
     */
    public double calculate(double reference, double state) {
        // 1. Calcular Error
        double error = reference - state;

        // 2. Calcular Delta Time
        double deltaTime = timer.seconds();
        timer.reset(); // Reiniciar el temporizador para la próxima medición [9]

        // 3. Calcular Integral [10]
        integralSum += (error * deltaTime);
        
        // Aplicar Anti-Windup (limitar la suma integral) [5]
        if (integralSum > INTEGRAL_SUM_LIMIT) {
            integralSum = INTEGRAL_SUM_LIMIT;
        } else if (integralSum < -INTEGRAL_SUM_LIMIT) {
            integralSum = -INTEGRAL_SUM_LIMIT;
        }

        // 4. Calcular Derivativo [11]
        double derivative = 0;
        if (deltaTime > 0) {
            derivative = (error - lastError) / deltaTime;
        }
        
        // 5. Calcular Feedforward (proporcional a la referencia, crucial para velocidad) [7]
        double feedForward = reference * Kf;

        // 6. Calcular Salida total (P + I + D + FF)
        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + feedForward;

        // 7. Actualizar variables de estado
        lastError = error;

        // La SDK de FTC limitará (clip) la potencia a [-1, 1], 
        // pero el controlador debe calcular el valor ideal.
        return output; 
    }
}
