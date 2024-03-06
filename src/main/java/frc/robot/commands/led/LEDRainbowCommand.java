// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDRainbowCommand extends Command {
  private final LEDSubsystem ledSubsystem;
  private DoubleSupplier speedSupplier;
  private double position = 0;
  private int rainbowFirstPixelHue = 0; // Starting hue

  public LEDRainbowCommand(LEDSubsystem ledSubsystem) {
    addRequirements(ledSubsystem);

    this.ledSubsystem = ledSubsystem;
    this.speedSupplier = () -> 3;
  }

  public LEDRainbowCommand(LEDSubsystem ledSubsystem, double speed) {
    addRequirements(ledSubsystem);

    this.ledSubsystem = ledSubsystem;
    this.speedSupplier = () -> speed;
  }

  public LEDRainbowCommand(LEDSubsystem ledSubsystem, DoubleSupplier speedSupplier) {
    addRequirements(ledSubsystem);

    this.ledSubsystem = ledSubsystem;
    this.speedSupplier = speedSupplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (var i = 0; i < ledSubsystem.getBufferLength(); i++) {
      // Calculate the hue - distance along the strip
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledSubsystem.getBufferLength())) % 180;
      ledSubsystem.setPixelHSV(i, hue, 255, 128);
    }
    // Increase for next iteration to make it move
    rainbowFirstPixelHue = (int) Math.floor(position);
    // Wrap the hue
    rainbowFirstPixelHue %= 180;

    position += speedSupplier.getAsDouble();
    if (position > 1000) {
      position -= 1000;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
