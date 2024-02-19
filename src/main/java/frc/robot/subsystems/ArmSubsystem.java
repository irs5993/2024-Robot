package frc.robot.subsystems;

import frc.robot.Constants.CANIDS;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANcoder encoder;

    private static final double ARM_MIN = 0.433;
    private static final double ARM_MAX = 0.62;

    public ArmSubsystem() {
        leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);

        encoder = new CANcoder(CANIDS.ARM_ENCODER);
    }

    public double getEncoderAbsolutePosition() {
        // Return the current position of the arm from the encoder
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Value", getEncoderAbsolutePosition());
    }

    public void setMotorSpeed(double speed) {
        if (Math.abs(speed) > 0.6) {return;}
                // System.out.println(speed);


        if (speed < 0 && getEncoderAbsolutePosition() < ARM_MIN) {
            return;
        }
        if (speed > 0 && getEncoderAbsolutePosition() > ARM_MAX) {
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
