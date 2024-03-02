// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.helpers.RMath;

public class TurnAngleCommand extends PIDCommand {
  public TurnAngleCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier angleSupplier) {
    super(
        // The controller that the command will use
        new PIDController(0.06, 0, 0.01),
        // This should return the measurement
        drivetrainSubsystem::getYaw,
        // This should return the setpoint (can also be a constant)
        () -> RMath.convertToRobotAngle(angleSupplier.getAsDouble()),
        // This uses the output
        output -> {
          drivetrainSubsystem.drive(0, MathUtil.clamp(output, -0.4, 0.7));
        });

    addRequirements(drivetrainSubsystem);

    getController().setTolerance(2);
    getController().enableContinuousInput(-180, 180);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}