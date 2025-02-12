// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import java.lang.Thread;
import java.util.Optional;
import java.util.concurrent.locks.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableQuerier {

  public static class TagCollection {
    private NetworkTableEntry ids_;
    private NetworkTableEntry distances_;
    private NetworkTableEntry azimuths_;
    private NetworkTableEntry elevations_;
    private NetworkTableEntry offsets_;
    private NetworkTableEntry rotations_;

    // for when we want to hold concurrency
    public ReadWriteLock lock = new ReentrantReadWriteLock();

    /**
     * Tag IDs. These are longs instead of ints because that's what goes in the network table
     */
    public long[] ids;
    /**
     * Distances to the april tags. THESE ARE IN INCHES.
     */
    public double[] distances;
    /**
     * Azimuth angles to the april tags, in radians.
     */
    public double[] azimuths;
    /**
     * Elevation angles to the april tags, in radians.
     */
    public double[] elevations;
    /**
     * Offset distances to the april tags, IN INCHES.
     * Offset is the amount left or right the center of the image is on the plane parallel with the tag.
     */
    public double[] offsets;
    /**
     * Rotations of the tags themselves.
     */
    public double[] rotations;

    public TagCollection(NetworkTable collection) {
      ids_ = collection.getEntry("ids");
      distances_ = collection.getEntry("d");
      azimuths_ = collection.getEntry("a");
      elevations_ = collection.getEntry("e");
      offsets_ = collection.getEntry("o");
      rotations_ = collection.getEntry("r");
    }

    /**
     * Refresh the values. Doesn't take into account any synchronization.
     */
    public void refresh() {
      ids = ids_.getIntegerArray(ids);
      distances = distances_.getDoubleArray(distances);
      azimuths = azimuths_.getDoubleArray(azimuths);
      elevations = elevations_.getDoubleArray(elevations);
      offsets = offsets_.getDoubleArray(offsets);
      rotations = rotations_.getDoubleArray(rotations);
    }

    /**
     * Try to refresh, holding a lock to the container. Does nothing if the lock is held.
     * 
     * @return whether we successfully refreshed.
     */
    public boolean tryRefresh() {
      Lock write = lock.writeLock();
      if (write.tryLock()) {
        try {
          refresh();
          return true;
        } finally {
          write.unlock();
        }
      } else return false;
    }
    /**
     * Refresh, waiting until the lock is available.
     */
    public void syncRefresh() {
      Lock write = lock.writeLock();
      write.lock();
      try {
        refresh();
      } finally {
        write.unlock();
      }
    }

    /**
     * Get the index of a tag in this collection.
     * 
     * @param id the tag ID we want
     * @return an optional value if a tag is found
     */
    public Optional<Integer> tagIndex(int id) {
      for (int i = 0; i < ids.length; ++i) {
        if (ids[i] == id) return Optional.of(i);
      }
      return Optional.empty();
    }
  }

  // Create network tables
  private NetworkTableInstance networkTableInstance;

  private NetworkTable controlTable;

  // Create network table entries
  private NetworkTableEntry robotStop;
  private NetworkTableEntry zeroGyro;
  private NetworkTableEntry colorSelection;

  private boolean runNetworkTables;

  public class Runner implements Runnable {
    @Override
    public void run() {
      while (runNetworkTables) {
        queryNetworkTables();
      }
    }
  }

  /**
   * Class constructor
   */
  public NetworkTableQuerier() {

    // Initialize the network tables
    initNetworkTables();

    // Set flags
    runNetworkTables = true;
  }

  /**
   * Start the main execution thread
   */
  public void start() {
    runNetworkTables = true;
    Thread ntThread = new Thread(new Runner());
    ntThread.setDaemon(true);
    ntThread.setName("NT-query");
    ntThread.start();
  }

  /**
   * Stop the main execution thread
   */
  public void stop() {
    runNetworkTables = false;
  }

  /**
   * Initialize network tables
   */
  private void initNetworkTables() {

    networkTableInstance = NetworkTableInstance.getDefault();
    controlTable = networkTableInstance.getTable("control");

    robotStop = controlTable.getEntry("RobotStop");
    zeroGyro = controlTable.getEntry("ZeroGyro");
    colorSelection = controlTable.getEntry("ColorSelection");

    robotStop.setNumber(0);
    zeroGyro.setNumber(0);

    queryNetworkTables();
  }

  /**
   * Get values from network tables
   */
  private void queryNetworkTables() {
    robotStop = controlTable.getEntry("RobotStop");

    colorSelection = controlTable.getEntry("BallColor");
  }

  /**
   * Set the robot stop flag
   */
  public synchronized void robotStop() {
    robotStop.setNumber(1);
  }

  public synchronized void setColor(int color) {
    colorSelection.setNumber(color);
  }
}