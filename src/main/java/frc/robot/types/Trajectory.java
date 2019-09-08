package frc.robot.types;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.io.File;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.IOException;

public class Trajectory {
    public int currentIndex = 0;
    public ArrayList<PathPoint> points;

    public class PathPoint {
        public double position;
        public double velocity;
        public double acceleration;

        PathPoint(double pos, double vel, double acc) {
            position = pos;
            velocity = vel;
            acceleration = acc;
        }

        Double[] toArray() {
            return new Double[] {position, velocity, acceleration};
        }
    }

    public Trajectory(String filePath) throws IOException {
        BufferedReader csvReader = new BufferedReader(new FileReader(filePath));
        csvReader.readLine();
        String row;
        while ((row = csvReader.readLine()) != null) {
            String[] data = row.split(",");
            double pos = Double.parseDouble(data[0]);
            double vel = Double.parseDouble(data[1]);
            double acc = Double.parseDouble(data[2]);
            points.add(new PathPoint(pos, vel, acc));
        }
        csvReader.close();
    }

    public Trajectory(File file) throws IOException {
        BufferedReader csvReader = new BufferedReader(new FileReader(file));
        String row;
        csvReader.readline();
        while ((row = csvReader.readLine()) != null) {
            String[] data = row.split(",");
            double pos = Double.parseDouble(data[0]);
            double vel = Double.parseDouble(data[1]);
            double acc = Double.parseDouble(data[2]);
            points.add(new PathPoint(pos, vel, acc));
        }
        csvReader.close();
    }

    public boolean hasNext() {
        return currentIndex >= points.size();
    }

    public PathPoint getNext() {
        if (!hasNext()) {
            throw new NoSuchElementException();
        }
        return points.get(currentIndex++);
    }

    public Double[] getNextValues() {
        if (!hasNext()) {
            throw new NoSuchElementException();
        }
        return points.get(currentIndex++).toArray();
    }
}
