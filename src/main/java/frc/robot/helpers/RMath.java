package frc.robot.helpers;

import java.util.function.DoubleSupplier;

public class RMath {
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double convertToRobotAngle(double povAngle) {
        // Ensure the angle is within the range [0, 360)
        double normalizedAngle = (povAngle % 360 + 360) % 360;

        // Convert to the range [-180, 180)
        double robotAngle = normalizedAngle > 180 ? normalizedAngle - 360 : normalizedAngle;

        return robotAngle;
    }
}