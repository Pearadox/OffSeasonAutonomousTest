package frc.lib.pathfollowing;

public class Point {
  public double position;
  public double velocity;
  public double acceleration;
  public double heading;

  Point(double pos, double vel, double acc, double hea) {
    position = pos;
    velocity = vel;
    acceleration = acc;
    heading = hea;
  }
}