// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import java.lang.Thread;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class NetworkTableQuerier implements Runnable {

    // Create network tables
    private static NetworkTableInstance networkTableInstance;
    private static NetworkTable controlTable;
    private static NetworkTable cam1Table;
    private static NetworkTable cam2Table;

    // Create network table entries
    private static NetworkTableEntry robotStop;
    private static NetworkTableEntry timeString;
    private static NetworkTableEntry zeroGyro;
    private static NetworkTableEntry targetLock;
    private static NetworkTableEntry colorSelection;


    // Declare class variables
    private boolean runNetworkTables;


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
     * Main execution thread
     */
    public void run() {
        
        while (runNetworkTables) {

            queryNetworkTables();
             
        }
    }


    /**
     * Start the main execution thread
     */
    public void start() {

        runNetworkTables = true;
        Thread ntThread = new Thread(this);
        ntThread.start();

    }


    /**
     * Stop the main execution thread
     */
    public void stop() {

        runNetworkTables = false;
        
    }

    public NetworkTable getControlTable() {
        return controlTable;
    }

    /**
     * Initialize network tables
     */
    private void initNetworkTables() {

        networkTableInstance = NetworkTableInstance.getDefault();
        controlTable = networkTableInstance.getTable("control");
        cam1Table = networkTableInstance.getTable("CAM1");
        cam2Table = networkTableInstance.getTable("CAM2");

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

        targetLock = controlTable.getEntry("TargetLock");
        colorSelection = controlTable.getEntry("BallColor");



        SmartDashboard.putBoolean("TargetLock", targetLock.getBoolean(false));
    }

    /*
     * @param entry The ID of the NetworkTables entry to return
     * @return the double value of the NetworkTables entry chosen; an error will be returned if entry is not a double 
     * 
     * List of available entries:
     * "BallDistance"
     * "BallAngle"
     * "BallScreenPercent"
     * "TapeOffset"
     * "TapeDistance" 
     */
    public synchronized double getControlDouble(String entry) {

        return controlTable.getEntry(entry).getDouble(0);

    }

    public synchronized void putControlDouble(String entry, double value) {
        controlTable.getEntry(entry).setDouble(value);
    }

    public synchronized void putControlString(String entry, String value) {
        controlTable.getEntry(entry).setString(value);
    }

    /*
     * @param entry The ID of the NetworkTables entry to return
     * @return the boolean value of the NetworkTables entry chosen; an error will be returned if entry is not a boolean 
     * 
     * List of available entries:
     * "FoundBall"
     * "FoundTape"
     * "TargetLock" 
     */
    public synchronized boolean getVisionBoolean(String camera, String entry) {

        boolean camValue = false;

        if (camera == "Cam1") {
            camValue = cam1Table.getEntry(entry).getBoolean(false);
        } else{
            camValue = cam2Table.getEntry(entry).getBoolean(false);
        }
        return camValue;
    }

    public synchronized double getVisionDouble(String camera, String entry) {

        double camValue = 0;

        if (camera == "Cam1") {
            camValue = cam1Table.getEntry(entry).getDouble(0);
        } else{
            camValue = cam2Table.getEntry(entry).getDouble(0);
        }
        return camValue;
    }

    public synchronized double getRingsFound(String camera) {

        double camValue = 0;

        if (camera == "Cam1") {
            camValue = cam1Table.getEntry("RingsFound").getDouble(0);
        } else{
            camValue = cam2Table.getEntry("RingsFound").getDouble(0);
        }
        return camValue;
    }

    public synchronized double getRingInfo(String camera, int ring, String entry) {

        double camValue = 0;

        if (camera == "Cam1") {
            camValue = cam1Table.getEntry("Rings." + ring + "." + entry).getDouble(0);
        } else{
            camValue = cam2Table.getEntry("Rings." + ring + "." + entry).getDouble(0);
        }
        return camValue;
    }

    public synchronized double getTagsFound(String camera) {

        double camValue = 0;

        if (camera == "CAM1") {
            System.out.println("Getting CAM1 Tags");
            camValue = cam1Table.getEntry("TagsFound").getDouble(0);
        } else{
            System.out.println("Getting CAM2 Tags");
            camValue = cam2Table.getEntry("TagsFound").getDouble(0);
        }
        return camValue;
    }

    public synchronized double getTagInfo(String camera, int tag, String entry) {

        double camValue = 0;
        String infoString = "Tags." + tag + "." + entry;
        System.out.println(infoString);

        if (camera == "CAM1") {
            camValue = cam1Table.getEntry(infoString).getDouble(0);
        } else{
            camValue = cam2Table.getEntry(infoString).getDouble(0);
        }
        return camValue;
    }


    /**
     * Get the Target Lock flag
     * @return
     */
    public synchronized boolean getTargetLockFlag() {

        return targetLock.getBoolean(false);

    }




    /**
     * Set the robot stop flag
     */
    public synchronized void robotStop() {

        robotStop.setNumber(1);
    }


    /**
     * Zero the VMX gyro
     */
    public synchronized void zeroPiGyro() {

        zeroGyro.setNumber(1);
    }

    public synchronized void setColor(int color)
    {
        colorSelection.setNumber(color);
    }
}