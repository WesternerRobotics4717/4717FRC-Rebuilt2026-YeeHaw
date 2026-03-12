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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
private final QuestNav questNav;
	private boolean qnavConnected = false;
	private boolean qnavTracking = false;
	private int qnavLostTrackingCount = -1;
	private int qnavBatteryPercent = -1;
	private double qnavLatency = -1;
	private Pose3d qnavRobotPose = new Pose3d();
	private Pose3d qnavRawPose = new Pose3d();
	private boolean qnavUseMeasurements;
	private double qnavFaultCounter = 0.0;
	private boolean qnavHealthy = false;

	private final PhotonCamera cam1;
    private final PhotonCamera cam2;
	private final PhotonPoseEstimator cam1Estimator;
    private final PhotonPoseEstimator cam2Estimator;
	private Matrix<N3, N1> pvStdDevs;
	private Pose3d pvRobotPose = new Pose3d();
	private boolean pvUseMeasurements;
	private boolean sniperConnected = false;
    private boolean nomConnected = false;
	private boolean pvHasTarget = false;
	private int pvBestTargetId = -1;

	/**
	 * Constructor for LocalizationSystem
	 * <p>
	 * Configures QuestNav and PhotonVision parts
	 */
	public LocalizationSystem() {
		// Set up QuestNav
		questNav = new QuestNav();
		qnavUseMeasurements = true;

		// Set up PhotonVision camera and estimator
		cam1 = new PhotonCamera(PV_CAM_ONE);
        cam2 = new PhotonCamera(PV_CAM_TWO);
		cam1Estimator = new PhotonPoseEstimator(
		  APRIL_TAG_FIELD_LAYOUT,
		  ROBOT_TO_SNIPER
		);

	    cam2Estimator = new PhotonPoseEstimator(
		  APRIL_TAG_FIELD_LAYOUT,
		  ROBOT_TO_NOM
		);

        

		pvUseMeasurements = false;


        

	}

	/**
	 * @return whether QuestNav headset is connected
	 */
	public boolean isQnavConnected() {
		return qnavConnected;
	}

	/**
	 * @return whether QuestNav headset is tracking
	 */
	public boolean isQnavTracking() {
		return qnavTracking;
	}

	/**
	 * @return count of times QuestNav tracking was lost since last restart
	 */
	public int getQnavLostTrackingCount() {
		return qnavLostTrackingCount;
	}

	/**
	 * @return current Quest battery percent
	 */
	public int getQnavBatteryPercent() {
		return qnavBatteryPercent;
	}

	/**
	 * @return current QuestNav connection latency in milliseconds
	 */
	public double getQnavLatency() {
		return qnavLatency;
	}

	/**
	 * @return the current estimated Pose3d from QuestNav WITH robot transform applied
	 */
	public Pose3d getQnavRobotPose() {
		return qnavRobotPose;
	}

	/**
	 * @return the current estimated Pose3d from QuestNav WITHOUT robot transform applied
	 */
	public Pose3d getQnavRawPose() {
		return qnavRawPose;
	}

	/**
	 * @return whether measurements from the Quest are currently being used
	 */
	public boolean getQnavUsingMeasurements() {
		return qnavUseMeasurements;
	}

	/**
	 * @return how many times QuestNav has disagreed with AprilTags
	 */
	public double getQnavFaultCounter() {
		return qnavFaultCounter;
	}

	/**
	 * @return whether faults are less than the declared threshold
	 */
	public boolean getQnavHealthy() {
		return qnavHealthy;
	}

	/**
	 * RETURNS NULL IF NO TAGS PRESENT!!!
	 *
	 * @return current estimated Pose3d from PhotonVision
	 */
	public Pose3d getPvRobotPose() {
		return pvRobotPose;
	}

	/**
	 * @return whether PhotonVision camera is connected
	 */
	public boolean isSniperConnected() {
		return sniperConnected;
	}

    public boolean isNomConnected() {
        return nomConnected;
    }

	/**
	 * @return whether or not PhotonVision has a target
	 */
	public boolean getPvHasTarget() {
		return pvHasTarget;
	}

	/**
	 * @return the ID of the best AprilTag target, -1 if no targets
	 */
	public int getPvBestTargetId() {
		return pvBestTargetId;
	}

	/**
	 * Set the raw Quest pose, with NO robot offsets included.
	 *
	 * @param pose the Pose3d to set to
	 */
	public void setQnavRawPose(Pose3d pose) {
		questNav.setPose(pose);
	}

	/**
	 * Set the Quest-reported ROBOT pose. Offset applied automatically. Where do you want the robot to think it is?
	 *
	 * @param pose the Pose3d to set to
	 */
	public void setQnavRobotPose(Pose3d pose) {
		setQnavRawPose(pose.transformBy(ROBOT_TO_QUEST));
	}

	/**
	 * Pose2d version of this method. All other values set to zero. Check whether you need 3d positioning data.
	 *
	 * @param pose the Pose2d to set to
	 */
	public void setQnavRobotPose(Pose2d pose) {
		setQnavRobotPose(new Pose3d(pose));
	}

	/**
	 * Set whether Quest measurements are being used for pose estimation
	 */
	public void setQnavUseMeasurements(boolean b) {
		qnavUseMeasurements = b;
	}

	public Command enableQnavMeasurements() {
		return Commands.runOnce(() -> setQnavUseMeasurements(true)).ignoringDisable(true);
	}

	public Command disableQnavMeasurements() {
		return Commands.runOnce(() -> setQnavUseMeasurements(false)).ignoringDisable(true);
	}

	/**
	 * Set whether PhotonVision measurements are being used for pose estimation
	 */
	public void setPVUseMeasurements(boolean b) {
		pvUseMeasurements = b;
	}

	public Command enablePVMeasurements() {
		return Commands.runOnce(() -> setPVUseMeasurements(true)).ignoringDisable(true);
	}

	public Command disablePVMeasurements() {
		return Commands.runOnce(() -> setPVUseMeasurements(false)).ignoringDisable(true);
	}

	/**
	 * Internal method which dynamically updates standard deviations based on number of tags
	 *
	 * @param estimatedPose estimated robot pose
	 * @param targets List of PhotonTrackedTargets
	 */
	private void updateEstimationStdDevs(
	  Optional<EstimatedRobotPose> estimatedPose,
	  List<PhotonTrackedTarget> targets
	) {
		if (estimatedPose.isEmpty()) {
			// No pose input. Default to single-tag std devs
			pvStdDevs = VisionConstants.pvConstants.PV_SINGLE_TAG_STD_DEVS;
		} else {
			// Pose present. Start running heuristic.
			var estStdDevs = VisionConstants.pvConstants.PV_SINGLE_TAG_STD_DEVS;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation - see how many tags we count, and calculate an average-distance metric
			for (var tgt : targets) {
				var tagPose = cam1Estimator.getFieldTags().getTagPose(tgt.getFiducialId());
				if (tagPose.isEmpty()) continue;
				numTags++;
				avgDist +=
				  tagPose
					.get()
					.toPose2d()
					.getTranslation()
					.getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
			}

			if (numTags == 0) {
				// No tags visible. Default to single-tag std devs
				pvStdDevs = VisionConstants.pvConstants.PV_SINGLE_TAG_STD_DEVS;
			} else {
				// One or more tags visible, run the full heuristic.
				avgDist /= numTags;
				// Increase std devs if multiple targets are visible.
				if (numTags > 1) estStdDevs = VisionConstants.pvConstants.PV_MULTI_TAG_STD_DEVS;
				// Increase std devs based on (average) distance
				if (numTags == 1 && avgDist > 4) {
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				} else {
					estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
				}
				pvStdDevs = estStdDevs;
			}
		}
	}

	/**
	 * Sets QuestNav pose to PhotonVision pose, if tags are viewable
	 */
	public void syncPoses() {
		Pose3d p = getPvRobotPose();

		if (p != null) {
			setQnavRobotPose(p);
			qnavFaultCounter = 0;
			System.out.println("[i] QuestNav pose reset successfully");
		} else {
			System.out.println("[!] QuestNav pose reset failed");
		}

	}

	/**
	 * Handles periodic updates for QuestNav pose estimation.
	 * <p>
	 * Processes unread QuestNav pose frames, validates poses against AprilTag estimates,
	 * manages fault detection, and adds valid vision measurements.
	 *
	 * @param bestEstimate the best AprilTag pose estimate for comparison, or null if none available
	 */
	public void qnavPeriodic(Pose3d bestEstimate) {
		questNav.commandPeriodic(); // required by the headset

		// Update everything but poses first
		qnavConnected = questNav.isConnected();

		qnavTracking = questNav.isTracking();

		OptionalInt optionalInt = questNav.getTrackingLostCounter();
		qnavLostTrackingCount = optionalInt.isPresent() ? optionalInt.getAsInt() : -1;

		optionalInt = questNav.getBatteryPercent();
		qnavBatteryPercent = optionalInt.isPresent() ? optionalInt.getAsInt() : -1;

		qnavLatency = questNav.getLatency();

		boolean isQuestWorking = qnavConnected && qnavTracking;
		if (!isQuestWorking && qnavFaultCounter < QUESTNAV_FAILURE_THRESHOLD) {
			qnavFaultCounter++;
		}

		// Iterate backwards through frames to find the most recent valid frame
		PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
		for (int i = frames.length - 1; i >= 0; i--) {
			PoseFrame frame = frames[i];

			if (frame.isTracking()) {
				qnavRawPose = frame.questPose3d();
				qnavRobotPose = qnavRawPose.transformBy(ROBOT_TO_QUEST.inverse());

				double bestEstimateDistance = (bestEstimate != null)
				  ? qnavRobotPose.toPose2d().getTranslation().getDistance(bestEstimate.toPose2d().getTranslation())
				  : 0;

				if (bestEstimateDistance > QUESTNAV_APRILTAG_ERROR_THRESHOLD.in(Meters)) {
					//Logger.recordOutput("BestEstimateDistance", bestEstimateDistance);
					// QuestNav disagrees with AprilTag vision: increment fault counter
					if (qnavFaultCounter < QUESTNAV_FAILURE_THRESHOLD) {
						qnavFaultCounter += Math.pow(bestEstimateDistance, 2);
					}
				} else if (bestEstimate != null) {
					// QuestNav agrees with AprilTags - decrement counter to allow recovery
					qnavFaultCounter = Math.max(qnavFaultCounter - 1.0, 0.0);
				}

				qnavHealthy = qnavFaultCounter < QUESTNAV_FAILURE_THRESHOLD;

				// Only use QuestNav measurements when fault counter is below threshold
				// TODO: add field pose validation here
				if (qnavHealthy && qnavUseMeasurements) {
					RobotContainer.drive.addVisionMeasurement(
					  qnavRobotPose.toPose2d(),
					  frame.dataTimestamp(),
					  QNAV_STD_DEVS
					);
				}

				break; // found the most recent tracking frame, exit loop
			}
		}


	}

	public void pvPeriodic() {
		// Update everything but poses first
		sniperConnected = cam1.isConnected();
        nomConnected = cam2.isConnected();

		// Grab all unread results from camera
		List<PhotonPipelineResult> pvCamResults = cam1.getAllUnreadResults();
        List<PhotonPipelineResult> nomResults = cam2.getAllUnreadResults();

		// Process logging & variables if unread results found
		if (!pvCamResults.isEmpty()) {
			PhotonPipelineResult pvCamResult = pvCamResults.get(pvCamResults.size() - 1);

			pvHasTarget = pvCamResult.hasTargets();

			if (pvHasTarget) {
				pvBestTargetId = pvCamResult.getBestTarget().getFiducialId();
			} else {
				pvBestTargetId = -1;
			}
		}

		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (var change : pvCamResults) {
			// Attempt multi-tag estimation
			visionEst = cam1Estimator.estimateCoprocMultiTagPose(change);

			// If multi-tag fails, fall back to lowest ambiguity
			if (visionEst.isEmpty()) {
				visionEst = cam1Estimator.estimateLowestAmbiguityPose(change);
			}

			// Process standard deviations
			updateEstimationStdDevs(visionEst, change.getTargets());
		}

		if (visionEst.isPresent() && pvUseMeasurements) {
			pvRobotPose = visionEst.get().estimatedPose;

			// If QuestNav considered unhealthy, or not enabled, fall back to PhotonVision measurements
			if (!qnavHealthy || !qnavUseMeasurements) {
				RobotContainer.drive.addVisionMeasurement(
				  pvRobotPose.toPose2d(),
				  visionEst.get().timestampSeconds,
				  pvStdDevs
				);
			}
		}

		// Log all values
		
	}

	@Override
	public void periodic() {
		// PhotonVision periodic
		pvPeriodic();

		// QuestNav periodic
		qnavPeriodic(pvRobotPose);
	}

    }




