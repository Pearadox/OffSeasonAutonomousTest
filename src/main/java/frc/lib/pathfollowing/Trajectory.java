package frc.lib.pathfollowing;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Trajectory {
  private List<Point> points = new ArrayList<>();
  private int currentIndex;

  public Trajectory(String fileName) throws IOException {
    this(new File(fileName));
  }

  public Trajectory(File file) throws IOException {
    BufferedReader csvReader = new BufferedReader(new FileReader(file));
    String row;
    while ((row = csvReader.readLine()) != null) {
      String[] data = row.split(",");
      double pos = Double.parseDouble(data[0]);
      double vel = Double.parseDouble(data[1]);
      double acc = Double.parseDouble(data[2]);
      double hea = Double.parseDouble(data[3]);
      points.add(new Point(pos, vel, acc, hea));
    }
    csvReader.close();
  }

  public Point next() {
    return points.get(currentIndex++);
  }

  public Point first() {
    return points.get(0);
  }

  public boolean hasNext() {
    return currentIndex < points.size();
  }

  public double getTotalTime() {
    return points.size() * .2;
  }
}