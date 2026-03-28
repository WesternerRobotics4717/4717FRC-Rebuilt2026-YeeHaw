package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShotMap;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AaronAllianceFlip;
import org.littletonrobotics.junction.Logger;

// Does NOT shoot while aiming
public class AutoAim extends Command {

  private final Drive drive;
  private final ShotMap shotMap;
  private Translation2d hubPose = ShooterConstants.blueHubCenter;

  public AutoAim(Drive drive, ShotMap shotMap) {
    this.drive = drive;
    this.shotMap = shotMap;
    this.addRequirements(drive);
  }

  // Fix Target Disttance issue

  @Override
  public void initialize() {

    if (AaronAllianceFlip.shouldFlip()) {
      hubPose = ShooterConstants.redHubCenter;
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("AutoAim/HubPose", hubPose.getX());
    Pose2d robotPose = drive.getPose();

    // Distance to target
    double distance = robotPose.getTranslation().getDistance(hubPose);
    SmartDashboard.putNumber("AutoAim/TargetDistance", distance);

    // Get shot values
    double targetAngle = shotMap.getGoodHoodAngle(distance);
    double targetRPM = shotMap.getGoodRPM(distance);

    // 2D angle to target (for drivetrain rotation)
    Rotation2d angleToTarget =
        new Rotation2d(hubPose.getX() - robotPose.getX(), hubPose.getY() - robotPose.getY());

    Logger.recordOutput("AutoAim/HoodAngle", targetAngle);
    Logger.recordOutput("AutoAim/RPM", targetRPM);

    drive.rotateTo((angleToTarget));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
