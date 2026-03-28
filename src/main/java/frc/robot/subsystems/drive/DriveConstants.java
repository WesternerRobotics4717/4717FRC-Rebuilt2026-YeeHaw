package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class DriveConstants {
  public static final int frontLeftDriveCanId = 11;
  public static final int backLeftDriveCanId = 13;
  public static final int frontRightDriveCanId = 12;
  public static final int backRightDriveCanId = 14;

  public static final int frontLeftTurnCanId = 21;
  public static final int backLeftTurnCanId = 23;
  public static final int frontRightTurnCanId = 22;
  public static final int backRightTurnCanId = 24;

  public static final int frontLeftCANcoderCanId = 31;
  public static final int backLeftCANcoderCanId = 33;
  public static final int frontRightCANcoderCanId = 32;
  public static final int backRightCANcoderCanId = 34;

  public static Pose3d blueOutpost =
      new Pose3d(0.8382, 0.457, 0, new Rotation3d(0, 0, 1.5 * Math.PI));
  public static Pose3d redOutpost =
      new Pose3d(16.12265, 7.624826, 0, new Rotation3d(0, 0, 1.5 * Math.PI));

  // General and Random

  public static final int climbCanId = 53;
  public static final int pigeonCanId = 2;
}
