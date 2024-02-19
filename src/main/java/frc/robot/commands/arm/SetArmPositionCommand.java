// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends PIDCommand {
  public SetArmPositionCommand(ArmSubsystem armSubsystem, DoubleSupplier positionSupplier) {
    super(
        // The controller that the command will use
        new PIDController(0.05, 0, 0.01),
        // This should return the measurement
        () -> armSubsystem.getEncoderAbsolutePosition(),
        // This should return the setpoint (can also be a constant)
        positionSupplier::getAsDouble,
        // This uses the output
        output -> {
          armSubsystem.setMotorSpeed(output);
        });
        addRequirements(armSubsystem);
        
        // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}