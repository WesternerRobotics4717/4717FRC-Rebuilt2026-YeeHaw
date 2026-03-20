package frc.robot.commands;

import frc.robot.subsystems.IndexTake.Intake;
import frc.robot.subsystems.IndexTake.Indexer;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.ShotMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {

    private final Turret turret;
    private final Intake intake;
    private final Drive drive;
    private final Hood hood;
    private final ShotMap shotMap;

    private final Pose2d targetPose;

    public AutoAim(
        Turret turret,
        Intake intake,
        Drive drive,
        Hood hood,
        ShotMap shotMap,
        Pose2d targetPose
    ) {
        this.turret = turret;
        this.intake = intake;
        this.drive = drive;
        this.hood = hood;
        this.shotMap = shotMap;
        this.targetPose = targetPose;

        addRequirements(turret, intake, drive);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();

        // Distance to target
        double distance = robotPose.getTranslation()
            .getDistance(targetPose.getTranslation());

        // Get shot values
        double targetAngle = shotMap.getGoodHoodAngle(distance);
        double targetRPM = shotMap.getGoodRPM(distance);

        // 2D angle to target (for drivetrain rotation)
        Rotation2d angleToTarget = new Rotation2d(
            targetPose.getX() - robotPose.getX(),
            targetPose.getY() - robotPose.getY()
        );

        // Command systems
        hood.setHoodAngle(targetAngle);
        turret.setRPMs(targetRPM);

        drive.rotateTo(angleToTarget);
    }
}