package frc.robot.subsystems.cameras;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

public class CameraBuilder {

  public int dev;
  public String name;
  public int fps_ = 10;
  public int brightness_ = 50;
  public int width_ = 160;
  public int height_ = 120;

  // Create a camera. Each camera needs to have a port and a name.
  public CameraBuilder(int dev, String name) {
    this.dev = dev;
    this.name = name;
  }
  
  // If a name is not specified, generate a generic one.
  public CameraBuilder(int dev) {
    this.dev = dev;
    this.name = "Camera " + dev;
  }

  // Set the FPS of the camera.
  public CameraBuilder fps(int fps) {
    this.fps_ = fps;
    return this;
  }

  // Set the brightness of the camera.
  public CameraBuilder brightness(int brightness) {
    this.brightness_ = brightness;
    return this;
  }

  // Set the resolution of the camera.
  public CameraBuilder resolution(int width, int height) {
    this.width_ = width;
    this.height_ = height;
    return this;
  }

  // Set the resolution of the camera; a shorter version of `resolution`.
  public CameraBuilder res(int width, int height) {
    this.width_ = width;
    this.height_ = height;
    return this;
  }

  // Finalize construction. All of the initialization is done here, so a CameraBuilder can be dropped before this is called with no consequence.
  public UsbCamera finish() {
    UsbCamera cam = new UsbCamera(name, dev);
    cam.setBrightness(brightness_);
    cam.setFPS(fps_);
    cam.setResolution(width_, height_);
    cam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    return cam;
  }

  // Finalize construction and attach to the camera server with CameraServer.AddCamera. When this is done, CameraServer.addCamera is called, which should(?) make it stream video.
  public UsbCamera attachToServer() {
    UsbCamera cam = new UsbCamera(name, dev);
    cam.setBrightness(brightness_);
    cam.setFPS(fps_);
    cam.setResolution(width_, height_);
    cam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    CameraServer.addCamera(cam);
    return cam;
  }

  // Finalize construction and attach to the camera server with CameraServer.startAutomaticCapture. When this is done, CameraServer.addCamera is called, which should(?) make it stream video.
  public UsbCamera attachAutoCapture() {
    UsbCamera cam = CameraServer.startAutomaticCapture(name, dev);
    cam.setBrightness(brightness_);
    cam.setFPS(fps_);
    cam.setResolution(width_, height_);
    return cam;
  }
};
