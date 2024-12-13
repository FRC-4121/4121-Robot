package frc.robot;

/**
 * Miscellaneous utilities 
 */
public final class Utils {
  /**
   * 
   * Convert a WPI angle (+/-180) to a CTRE angle (0-360)
   * 
   * @param angle The WPI angle to convert
   * @return A gyro angle
   * 
   */
  public static double fromWPIAngle(double angle) {
    if (angle > 0) {
      angle = (180 - angle) + 180;
    } else if (angle < 0) {
      angle = -angle;
    }
    return angle;
  }

  /**
   * 
   * Convert a CTRE angle (0-360) into a WPI angle (+/-180)
   * 
   * @param angle The CTRE angle to convert
   * @return A WPI based angle
   * 
   */
  public static double toWPIAngle(double angle) {
    if (angle > 180) {
      angle = -(angle - 360);
    } else if (angle > 0 && angle <=180) {
      angle = -angle;
    }
    return angle;
  }
}
