// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.cameras;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSwitcher extends SubsystemBase {

  private UsbCamera cams[];
  private VideoSink switcher;
  private int camIdx;

  public CameraSwitcher(UsbCamera[] cams, String name) {
    this.cams = cams;
    camIdx = 0;
    switcher = CameraServer.addSwitchedCamera(name);
    if (cams.length > 0) switcher.setSource(this.cams[0]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Switch to the next camera. This loops around to the start if called on the last camera.
  // If there are no cameras, then this is a no-op
  public void switchCamera() {
    if (cams.length > 0) {
      ++camIdx;
      camIdx %= cams.length;
      switcher.setSource(cams[camIdx]); // You're lucky I didn't make this a one-liner, Seth
    }
  }

  // Switch the camera to the designated source. If `idx` is out of bounds, nothing happens.
  public void switchCamera(int idx) {
    if (idx >= 0 && idx < cams.length) {
      camIdx = idx;
      switcher.setSource(cams[camIdx]);
    }
  }
}