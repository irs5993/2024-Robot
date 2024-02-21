package frc.robot.subsystems;

import frc.robot.Constants.CANIDS;
import frc.robot.helpers.RMath;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANcoder encoder;

    private static final double ARM_MIN = 0.004;
    private static final double ARM_MAX = 0.2;

    public ArmSubsystem() {
        leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);

        encoder = new CANcoder(CANIDS.ARM_ENCODER);
    }

    public double getEncoderAbsolutePosition() {
        // Return the current position of the arm from the encoder
        double value = encoder.getAbsolutePosition().getValueAsDouble();

        if (value > 0.9) {
            value = 0;
        }

        return value;
    }

    public double getAngle() {
        return encoderPositionToAngle(getEncoderAbsolutePosition());
    }

    public double encoderPositionToAngle(double position) {
        return RMath.map(position, ARM_MIN, ARM_MAX, 0, 90);

    }

    public double angleToEncoderPosition(double angle) {
        return RMath.map(angle, 0, 90, ARM_MIN, ARM_MAX);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Value", getEncoderAbsolutePosition());
        SmartDashboard.putNumber("Arm Angle", getAngle());

    }

    public void setMotorSpeed(double speed) {
        if (Math.abs(speed) > 0.75) {
            stop();
            return;
        }
        // System.out.println(speed);

        if (speed < 0 && getEncoderAbsolutePosition() < ARM_MIN) {
            stop();
            return;
        }
        if (speed > 0 && getEncoderAbsolutePosition() > ARM_MAX) {
            stop();
            return;
        }

        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
