package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexTake.Indexer;
import frc.robot.subsystems.IndexTake.Intake;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.ShotMap;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

// Does NOT shoot while aiming
public class AutoAim extends Command {

  private final Turret turret;
  private final Indexer indexer;
  private final Intake intake;
  private final Drive drive;
  private final Hood hood;
  private final ShotMap shotMap;

  private final Translation2d hubCenter = new Translation2d(4.625594, 4.034536);

  public AutoAim(
      Intake intake, Drive drive, Hood hood, Turret turret, Indexer indexer, ShotMap shotMap) {
    this.intake = intake;
    this.drive = drive;
    this.hood = hood;
    this.turret = turret;
    this.indexer = indexer;
    this.shotMap = shotMap;

    addRequirements(intake, drive);
  }

  // Fix Target Disttance issue

  @Override
  public void initialize() {
    //  if (DriverStation.getAlliance().isPresent() &&
    // DriverStation.getAlliance().get().equals(Alliance.Red)) {
    // hubCenter = Translation2d(4.625594, 4.034536);} else  {hubCenter = Translation2d()}
  }

  @Override
  public void execute() {

    Pose2d robotPose = drive.getPose();

    // Distance to target
    double distance = robotPose.getTranslation().getDistance(hubCenter);
    SmartDashboard.putNumber("AutoAim/TargetDistance", distance);

    // Get shot values
    double targetAngle = shotMap.getGoodHoodAngle(distance);
    double targetRPM = shotMap.getGoodRPM(distance);

    // 2D angle to target (for drivetrain rotation)
    Rotation2d angleToTarget =
        new Rotation2d(hubCenter.getX() - robotPose.getX(), hubCenter.getY() - robotPose.getY());

    Logger.recordOutput("AutoAim/HoodAngle", targetAngle);
    Logger.recordOutput("AutoAim/RPM", targetRPM);
    Logger.recordOutput("AutoAim/RobotRotation", angleToTarget);

    // // Command systems
    // hood.hoodVoidMove(targetAngle);
    // turret.spinShooterVoid(targetRPM);
    // indexer.indexerVoid(6);
    // intake.runIntakeVoid(5);

    drive.rotateTo((angleToTarget));
  }

  @Override
  public void end(boolean interrupted) {
    hood.hoodVoidMove(0);
    turret.stopShooterVoid();
    indexer.stopIndexerVoid();
    intake.stopIntakeVoid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
