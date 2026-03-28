package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.pvConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AaronAllianceFlip;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Subsystem for the robot localization
 *
 * <p>This uses Questnav as the primary and Photon as the secondary to provide pose estimation and
 * field localization
 */
public class LocalizationSystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();
  public static Pose3d cornerPose = DriveConstants.blueOutpost;
  // 634.75 in X, 300.19 in Y
  // 16.12265, 7.624826

  private static final Matrix<N3, N1> QUEST_STD_DEVS = QNAV_STD_DEVS;

  public LocalizationSystem() {
    Pose3d robotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

    questNav.setPose(questPose);
  }

  public Command zeroQNav() {
    if (AaronAllianceFlip.shouldFlip()) {
      cornerPose = DriveConstants.redOutpost;
    }
    Pose3d cornerPose = new Pose3d(0.8382, 0.457, 0, new Rotation3d(0, 0, 1.5 * Math.PI));

    return this.run(() -> questNav.setPose(cornerPose.transformBy(ROBOT_TO_QUEST)))
        .alongWith(RobotContainer.drive.setPose(cornerPose.toPose2d()))
        .alongWith(Commands.print("ResetPose"));
  }

  public static Pose3d getCornerPose() {
    if (AaronAllianceFlip.shouldFlip()) {
      cornerPose = DriveConstants.redOutpost;
    } else cornerPose = DriveConstants.blueOutpost;
    return cornerPose;
  }

  @Override
  public void periodic() {

    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame frame : frames) {
      if (!frame.isTracking()) continue;

      Pose3d questPose = frame.questPose3d();

      double timestamp = frame.dataTimestamp();

      Pose3d robotPose =
          questPose.transformBy(ROBOT_TO_QUEST.inverse()); // Added corner pose condition

      RobotContainer.drive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUEST_STD_DEVS);
    }
  }
}
