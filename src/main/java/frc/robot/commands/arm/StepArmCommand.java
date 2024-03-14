// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class StepArmCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final double speed;
  private double angle;

  public StepArmCommand(ArmSubsystem armSubsystem, double speed) {
    addRequirements(armSubsystem);

    this.armSubsystem = armSubsystem;
    this.speed = speed;
  }

  // Komut başlangıçta planlandığında çağrılır.
  @Override
  public void initialize() {
    armSubsystem.resetController();

    angle = armSubsystem.getAngle();
  }

  // Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    angle += speed;

    double desiredPosition = armSubsystem.angleToPosition(angle);
    armSubsystem.setPosition(desiredPosition);
  }

  // Komut sona erdiğinde veya kesintiye uğradığında çağrılır.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false;
  }
}
