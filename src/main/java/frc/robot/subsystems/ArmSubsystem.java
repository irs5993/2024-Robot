package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CANIDS;
import frc.robot.helpers.RMath;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private final CANcoder encoder;

  private final PIDController controller;

  private final double DEFAULT_P = 11.6;
  private final double DEFAULT_I = 0.0001;
  private final double DEFAULT_D = 0.7;
  // Alternative slower PID values: 5, 0, 0.2

  LinearFilter filter = LinearFilter.singlePoleIIR(0.4, 0.02);

  public ArmSubsystem() {
    leftMotor = new CANSparkMax(CANIDS.ARM_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(CANIDS.ARM_RIGHT, MotorType.kBrushless);

    encoder = new CANcoder(CANIDS.ARM_ENCODER);
    encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

    controller = new PIDController(DEFAULT_P, DEFAULT_I, DEFAULT_D);
    controller.setTolerance(Constants.Arm.CONTROLLER_TOLERANCE);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", getAbsolutePosition());
    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putNumber("Arm Pos Absolute Raw", encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm Pos Raw", encoder.getPosition().getValueAsDouble());

    if (getAbsolutePosition() <= Constants.Arm.MIN_POSITION) {
      setPosition(Constants.Arm.MIN_POSITION);
    }

    if (getAbsolutePosition() > Constants.Arm.MAX_POSITION) {
      stop();
    }
  }

  public void setMotorSpeed(double speed) {
    if (Math.abs(speed) > Constants.Arm.SAFETY_MAX_VOLTAGE) {
      stop();
      return;
    }

    double position = getAbsolutePosition();

    if (speed < 0 && position <= Constants.Arm.MIN_POSITION) {
      return;
    }
    if (speed > 0 && position > Constants.Arm.MAX_POSITION) {
      return;
    }

    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void setPosition(double position) {
    double raw = controller.calculate(getAbsolutePosition(),
        MathUtil.clamp(position, Constants.Arm.MIN_POSITION, Constants.Arm.MAX_POSITION));
    double speed = MathUtil.clamp(raw, -0.45, 0.65);
    setMotorSpeed(speed);

    SmartDashboard.putNumber("Arm PID Output", speed);
    SmartDashboard.putNumber("Desired Position", position);
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
    double value = encoder.getPosition().getValueAsDouble();

    return filter.calculate(value);
  }

  public double getAngle() {
    return positionToAngle(getAbsolutePosition());
  }

  public double positionToAngle(double position) {
    return RMath.map(position, Constants.Arm.MIN_POSITION, Constants.Arm.MAX_POSITION, 0, 90);
  }

  public double angleToPosition(double angle) {
    return RMath.map(angle, 0, 90, Constants.Arm.MIN_POSITION, Constants.Arm.MAX_POSITION);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
