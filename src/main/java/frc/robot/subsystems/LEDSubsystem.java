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

  private int rainbowFirstPixelHue = 0;

  /** Yeni bir LEDSubsystem yaratır. */
  public LEDSubsystem() {
    // Tamponu yeniden kullan
    // Varsayılan uzunluk 60'tır, boş çıktıyı başlat
    // Uzunluğun ayarlanması pahalıdır, bu nedenle yalnızca bir kez ayarlayın,
    // ardından verileri güncelleyin
    led = new AddressableLED(4);
    ledBuffer = new AddressableLEDBuffer(41);
    led.setLength(ledBuffer.getLength());

    // Veriyi ayarlayın
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }

  public void rainbow() {
    for (var i = 0; i < getBufferLength(); i++) {
      // Renk tonu hesapla - şerit boyunca mesafe
      final var hue = (rainbowFirstPixelHue + (i * 180 / getBufferLength())) % 180;
      this.setPixelHSV(i, hue, 255, 128);
    }

    rainbowFirstPixelHue += 2;
    // Wrap the hue
    rainbowFirstPixelHue %= 180;
  }

  public void setRGB(int r, int g, int b) {
    for (var i = 0; i < getBufferLength(); i++) {
      setPixelRGB(i, r, g, b);
    }
  }

  public void setPixelRGB(int index, int r, int g, int b) {
    ledBuffer.setRGB(index, r, g, b);
  }

  public void setPixelHSV(int index, int h, int s, int v) {
    ledBuffer.setHSV(index, h, s, v);
  }

  public int getBufferLength() {
    return ledBuffer.getLength();
  }

}
