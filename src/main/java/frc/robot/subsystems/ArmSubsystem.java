package frc.robot.subsystems;

import frc.robot.Constants.CANIDS;
import frc.robot.helpers.RMath;

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

    private static final double MAX_VOLTAGE = 0.75;
    private static final double MIN_POSITION = 0.004;
    private static final double MAX_POSITION = 0.2;
    private static final double CONTROLLER_TOLERANCE = 0.001;

    private final PIDController controller;
    private final double DEFAULT_P = 5;
    private final double DEFAULT_I = 0;
    private final double DEFAULT_D = 0.2;

    public ArmSubsystem() {
        leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);

        encoder = new CANcoder(CANIDS.ARM_ENCODER);

        controller = new PIDController(DEFAULT_P, DEFAULT_I, DEFAULT_D);
        controller.setTolerance(CONTROLLER_TOLERANCE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getAbsolutePosition());
        SmartDashboard.putNumber("Arm Angle", getAngle());
    }

    public void setMotorSpeed(double speed) {
        if (Math.abs(speed) > MAX_VOLTAGE) {
            stop();
            return;
        }

        double position = getAbsolutePosition();

        if (speed < 0 && position < MIN_POSITION) {
            stop();
            return;
        }
        if (speed > 0 && position > MAX_POSITION) {
            stop();
            return;
        }

        leftMotor.set(speed);
        rightMotor.set(-speed);
    }

    public void setPosition(double position) {
        double raw = controller.calculate(getAbsolutePosition(), position);
        double speed = MathUtil.clamp(raw, -0.3, 0.65);
        setMotorSpeed(speed);

        SmartDashboard.putNumber("Arm PID Output", speed);
    }

    public void resetController() {
        controller.reset();
        controller.setPID(DEFAULT_P, DEFAULT_I, DEFAULT_D);
    }

    public boolean controllerAtSetpoint() {
        return controller.atSetpoint();
    }

    public void setControllerPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    public double getAbsolutePosition() {
        // Return the current position of the arm from the encoder
        double value = encoder.getAbsolutePosition().getValueAsDouble();

        if (value > 0.9) {
            value = 0;
        }

        return value;
    }

    public double getAngle() {
        return positionToAngle(getAbsolutePosition());
    }

    public double positionToAngle(double position) {
        return RMath.map(position, MIN_POSITION, MAX_POSITION, 0, 90);
    }

    public double angleToPosition(double angle) {
        return RMath.map(angle, 0, 90, MIN_POSITION, MAX_POSITION);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
