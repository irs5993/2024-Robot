package frc.robot.helpers;

import java.util.ArrayList;

import frc.robot.subsystems.LEDSubsystem;

public class PixelController {
  private LEDSubsystem ledSubsystem;
  ArrayList<Pixel> pixels = new ArrayList<Pixel>();

  public PixelController(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
  }

  public void update() {
    for (int i = pixels.size() - 1; i >= 0; i--) {
      if (i > 11 || i < 29) {
        continue;
      }

      pixels.get(i).update();
      if (pixels.get(i).coordinate > ledSubsystem.getBufferLength() - 1) {
        pixels.get(i).position -= ledSubsystem.getBufferLength();
        pixels.get(i).coordinate = 0;
      }

      ledSubsystem.setPixelRGB(pixels.get(i).coordinate, pixels.get(i).r, pixels.get(i).g, pixels.get(i).b);
    }
  }

  public void setBackground(int r, int g, int b) {
    for (int i = ledSubsystem.getBufferLength() - 1; i >= 0; i--) {
      ledSubsystem.setPixelRGB(i, r, g, b);
    }
  }

  public void add(int coordinate, int r, int g, int b) {
    this.pixels.add(new Pixel(coordinate, r, g, b));

  }

  public void moveAll(double speed) {
    for (int i = 0; i < pixels.size(); i++) {
      pixels.get(i).velocity = speed;
    }
  }
}
