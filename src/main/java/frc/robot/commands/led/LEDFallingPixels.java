// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.Pixel;
import frc.robot.helpers.PixelController;
import frc.robot.subsystems.LEDSubsystem;

public class LEDFallingPixels extends Command {
  private final LEDSubsystem ledSubsystem;
  private PixelController pixelController;

  public LEDFallingPixels(LEDSubsystem ledSubsystem) {
    addRequirements(ledSubsystem);

    this.ledSubsystem = ledSubsystem;
  }

  @Override
  public void initialize() {
    this.pixelController = new PixelController(ledSubsystem);
    for (int i = 0; i < 30; i++) {
      this.pixelController.add(i, 255, i * 5, 0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.pixelController.setBackground(3, 0, 0);
    this.pixelController.update();

    this.pixelController.moveAll(0.5);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}