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
  private int rainbowFirstPixelHue = 0; // Başlangıç renk tonu

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

// Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
@Override
  public void execute() {
    for (var i = 0; i < ledSubsystem.getBufferLength(); i++) {
      // Renk tonu hesapla - şerit boyunca mesafe
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledSubsystem.getBufferLength())) % 180;
      ledSubsystem.setPixelHSV(i, hue, 255, 128);
    }
    // Hareket sağlamak için bir sonraki yinelemede arttırma
    rainbowFirstPixelHue = (int) Math.floor(position);
    // Wrap the hue
    rainbowFirstPixelHue %= 180;

    position += speedSupplier.getAsDouble();
    if (position > 1000) {
      position -= 1000;
    }
  }

  // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false;
  }
}
