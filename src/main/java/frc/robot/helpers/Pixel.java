package frc.robot.helpers;

import edu.wpi.first.math.MathUtil;

public class Pixel {
  public int coordinate;
  public double position, velocity, acceleration;
  public int r, g, b;

  public Pixel(int coordinate, int r, int g, int b) {
    this.coordinate = coordinate;
    this.position = coordinate;
    this.r = r;
    this.g = g;
    this.b = b;
  }

  public void update() {
    this.velocity += this.acceleration;
    this.position += this.velocity;
    this.coordinate = (int) Math.floor(position);
  }
}
