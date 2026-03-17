package frc.robot.subsystems;

public class DeviceIDs {
  
  //General IDs
  public static final int pigeonCanId = 2;

  public static final class ShooterConstants {
    public static final int flyWheelCanId = 50;
    public static final int shooterRollersCanId = 51;
    public static final int hoodCanId = 52;
    public static final int climbCanId = 53;
    public static final int shooterHoodThroughBore = 1;
    public static final double conversionFactor = (1/8.0)*360.0;
} 

public static final class driveIDs {
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

}

public static final class IntakeConstants {
    public static final int intakeSpinCanId = 40;
    public static final int intakeMoveCanId = 41;
    public static final int indexerBottomCanId = 42;
    public static final int indexerTopCanID = 43;
    public static final double conversionFactor = 360.0/60.0;
}
  
}






