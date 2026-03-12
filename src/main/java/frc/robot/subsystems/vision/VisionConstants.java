package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionConstants {
    public static final class buttonConstants{
        public static final int QUEST_MEASUREMENT_SWITCH = 13;
		public static final int PHOTONVISION_MEASUREMENT_SWITCH = 14;
    }
    public static final class pvConstants{
        public static final String PV_CAM_ONE = "ShooterCamera";
        public static final String PV_CAM_TWO = "IntakeCamera";
        public static final Transform2d SHOOTER_OFFSET = new Transform2d(-Units.inchesToMeters(10), 0, Rotation2d.kZero);

		public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
		  -0.26545065,
		  0.18207046,
		  0,
		  new Rotation3d(
			0,
			0,
			Math.PI / 2
		  )
		);

		public static final Transform3d ROBOT_TO_SNIPER = new Transform3d(
		  -Units.inchesToMeters(5),
		  -Units.inchesToMeters(1.375),
		  Units.inchesToMeters(19),
		  new Rotation3d(
			0,
			Units.degreesToRadians(-56),
			0
		  )
		);

        public static final Transform3d ROBOT_TO_NOM = new Transform3d(
		  -Units.inchesToMeters(5),
		  -Units.inchesToMeters(1.375),
		  Units.inchesToMeters(19),
		  new Rotation3d(
			0,
			Units.degreesToRadians(-56),
			0
		  )
		);

		public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
		public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6116, 4.0213);
		public static final Translation2d BLUE_HUB_BACK = new Translation2d(5.2342, 4.0213);

		public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9014, 4.0213);
		public static final Translation2d RED_HUB_BACK = new Translation2d(11.3044, 4.0213);

		public static final Translation2d BLUE_TOWER_CENTER = new Translation2d(1.1434, 3.7457);
		public static final Translation2d RED_TOWER_CENTER = new Translation2d(15.3952, 4.3236);

		//public static final String PV_CAM_NAME = "Bcam9782";
		// The standard deviations of our estimated poses, which affect correction rate
		public static final Matrix<N3, N1> PV_SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 8);
		public static final Matrix<N3, N1> PV_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
		public static Matrix<N3, N1> QNAV_STD_DEVS = VecBuilder.fill(
		  0.02, // Trust down to 2 cm in X direction
		  0.02, // Trust down to 2 cm in Y direction
		  0.035 // Trust down to 2 degrees rotational
		);

		/**
		 * Gets the Translation2d of the current Alliance Hub
		 *
		 * @return alliance Hub coordinates
		 */
		public static Translation2d getHubTranslation() {
			Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

			if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
				return RED_HUB_CENTER;
			} else {
				return BLUE_HUB_CENTER;
			}
		}

		public static final double QUESTNAV_FAILURE_THRESHOLD = 6.0;

		/**
		 * The threshold for the error between the best AprilTag pose estimate and the QuestNav pose measurements for the
		 * QuestNav pose to be considered valid
		 */
		public static final Distance QUESTNAV_APRILTAG_ERROR_THRESHOLD = Meters.of(0.5);
    }

    
    
}
