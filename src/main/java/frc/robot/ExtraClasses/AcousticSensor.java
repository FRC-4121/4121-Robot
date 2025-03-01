// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Define an acoustic range sensor
 */
public class AcousticSensor {

    // Declare variables
    private final AnalogInput sensor;
    private MedianFilter filter;
    private double lastValue;
    private double distanceThreshold;
    
    /**
     * Create a new acoustic sensor
     */
    public AcousticSensor() {

        // Initialize the analog input for the sensor
        sensor = new AnalogInput(0);

        // Initialize the signal filter
        filter = new MedianFilter(10);

        // Initialize other variables
        lastValue = 0;
        distanceThreshold = 6.0;

    }

    /**
     * Get the current distance reading
     * 
     * @return  Distance in inches
     */
    public double getDistance(){
        
        // Read the raw sensor value
        double rawValue = sensor.getValue();

        // Convert the raw value to inches and smooth
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;
        filter.calculate(currentDistanceInches);

        // Show distance on dashboard
        SmartDashboard.putNumber("inches", currentDistanceInches);

        // Return distance
        return currentDistanceInches;
    }

    /**
     * Check for coral to pass by sensor
     * 
     * @return  Flag indicating coral transit
     */
    public boolean checkForCoral() {
        return false;
    }
}
