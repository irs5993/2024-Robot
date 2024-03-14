package frc.robot.helpers;

public class RMath {
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double convertToRobotAngle(double povAngle) {
        // Açının [0, 360) aralığında olduğundan emin olur
        double normalizedAngle = (povAngle % 360 + 360) % 360;

        // [-180, 180) aralığına dönüştürür
        double robotAngle = normalizedAngle > 180 ? normalizedAngle - 360 : normalizedAngle;

        return robotAngle;
    }
}