package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.pvConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Subsystem for the robot localization
 *
 * <p>This uses Questnav as the primary and Photon as the secondary to provide pose estimation and
 * field localization
 */

// TODO: verify vision working, and remove photon vision?
public class LocalizationSystem extends SubsystemBase {
  private final QuestNav questNav = new QuestNav();

  private static final Matrix<N3, N1> QUEST_STD_DEVS = QNAV_STD_DEVS;

  public LocalizationSystem() {
    Pose3d robotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

    questNav.setPose(questPose);
  }

  @Override
  public void periodic() {

    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame frame : frames) {
      if (!frame.isTracking()) continue;

      Pose3d questPose = frame.questPose3d();

      double timestamp = frame.dataTimestamp();

      Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

      RobotContainer.drive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUEST_STD_DEVS);
    }
  }
}
