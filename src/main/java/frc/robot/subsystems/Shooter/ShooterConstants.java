package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;

public class ShooterConstants {
  public static final int flyWheelCanId = 50;
  public static final int rollerCanId = 51;
  public static final int hoodCanId = 52;
  public static final int shooterHoodThroughBore = 1;
  public static final double conversionFactor = 45;
  public static final double allowedError = 300;
  public static final double restingShooterRPM = 1750.0;
  public static final Translation2d blueHubCenter = new Translation2d(4.625594, 4.034536);
  public static final Translation2d redHubCenter = new Translation2d(11.915394, 4.034536);

  // Hub Position in Meters (4.625594, 4.034536)
  // Forward is X, Left/Right is Y
}
