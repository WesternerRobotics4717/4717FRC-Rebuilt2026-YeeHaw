package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.pvConstants.*;
import static frc.robot.subsystems.drive.DriveConstants.robotMOI;
import static frc.robot.subsystems.vision.VisionConstants.buttonConstants;

import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;


/**
 * Subsystem for the robot localization 
 * 
 * This uses Questnav as the primary and Photon as the secondary to provide pose estimation and field localization
 */
public class LocalizationSystem extends SubsystemBase{
		private final QuestNav questNav = new QuestNav();

		private static final Matrix<N3, N1> QUEST_STD_DEVS=
			QNAV_STD_DEVS;


		

	public LocalizationSystem(){
		Pose3d robotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

		Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

		questNav.setPose(questPose);
	}

 	@Override
	public void periodic(){
		

		PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

		for (PoseFrame frame :frames){
			if(!frame.isTracking()) continue;
			
			Pose3d questPose = frame.questPose3d();

			double timestamp = frame.dataTimestamp();

			Pose3d robotPose = 
			questPose.transformBy(ROBOT_TO_QUEST.inverse());

			RobotContainer.drive.addVisionMeasurement(
				robotPose.toPose2d(),
				timestamp,
				QUEST_STD_DEVS
			);
		}
	}
}





