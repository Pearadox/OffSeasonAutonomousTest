/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.logging;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * Add your docs here.
 */
public class CSVLogger {
    private final String loggingLocation;
    private final long period;
    private double totalTime = 0;
    private boolean running = false;
    private final File logFile;
    private FileWriter fileWriter;

    private final List<Supplier<String>> sources = new ArrayList<>();
    private final List<String> sourceNames = new ArrayList<>();

    public CSVLogger() throws IOException {
        this(1000);
    }

    public CSVLogger(long period) throws IOException {
        this(period, LocalDateTime.now().format(DateTimeFormatter.ISO_DATE));
    }

    public CSVLogger(long period, String fileName) throws IOException {
        this.period = period;
        if (new File("/media/sda1").exists()) {
            loggingLocation = "/media/sda1/logs";
        } else { loggingLocation = "/home/lvuser/logs"; }

        File logDirectory = new File(loggingLocation);
        if (!logDirectory.exists()) {
            try {
                Files.createDirectory(Paths.get(loggingLocation));
            } catch (IOException e) {
                throw new IOException("Directory");
            }
        }

        logFile = new File(loggingLocation + fileName + ".csv");
        if (!logFile.exists()) {
            try {
                Files.createFile(logFile.toPath());
            } catch (IOException e) {
                throw new IOException("File");
            }
        }

        addSource("Elapsed Time", () -> totalTime);
        fileWriter = new FileWriter(logFile);
        try {
            writeTitles();
        } catch (IOException e) {
            throw new IOException("Write");
        }
    }

    public synchronized void addSource(String name, Supplier<Object> supplier) {
        if (!running) {
            sourceNames.add(name);
            sources.add(supplier.get()::toString);
        } else {
            throw new IllegalStateException();
        }
    }

    public synchronized void start() {
        if (running) {
            throw new IllegalStateException();
        } else {
            running = true;
            Timer timer = new Timer();
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    try {
                        writeValues();
                    } catch (IOException e) {
                        e.printStackTrace();
                        cancel();
                    }
                    incrementTime();
                    if (!running) { 
                        timer.cancel();
                        timer.purge(); 
                    }
                }
            }, period);
        }
    }

    public void stop() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    private void writeTitles() throws IOException {
        fileWriter.write(sourceNames.stream().collect(Collectors.joining(",")) + "\n");
    }

    private void writeValues() throws IOException {
        fileWriter.write(sources.stream().map(s -> s.get()).collect(Collectors.joining(",")) + "\n");
    }

    private synchronized void incrementTime() {
        totalTime += ((double) period / 1000.0d);
    }
}