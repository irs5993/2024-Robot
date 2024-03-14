// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDLockedCommand extends Command {
  private final LEDSubsystem ledSubsystem;

  public LEDLockedCommand(LEDSubsystem ledSubsystem) {
    addRequirements(ledSubsystem);

    this.ledSubsystem = ledSubsystem;
  }

// Komut planlanırken zamanlayıcı her çalıştığında çağrılır.
  @Override
  public void execute() {
    for (var i = 0; i < ledSubsystem.getBufferLength(); i++) {
      ledSubsystem.setPixelRGB(i, 0, 255, 0);
    }
  }

 // Komutun bitmesi gerektiğinde true değerini döndürür.
  @Override
  public boolean isFinished() {
    return false;
  }
}
