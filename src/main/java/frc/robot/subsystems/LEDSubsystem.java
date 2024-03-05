// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    led = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }

  public void setRGB(int index, int r, int g, int b) {
    ledBuffer.setRGB(index, r, g, b);
  }

  public void setHSV(int index, int h, int s, int v) {
    ledBuffer.setHSV(index, h, s, v);
  }

  public int getBufferLength() {
    return ledBuffer.getLength();
  }

}
