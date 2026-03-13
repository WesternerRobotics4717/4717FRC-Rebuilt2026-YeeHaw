package frc.robot.subsystems.drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommands;
import frc.robot.game.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

import static frc.robot.subsystems.drive.DriveConstants.driveBaseRadius;

import org.littletonrobotics.junction.Logger;


public class AutoAim extends Command{
    private final CommandXboxController driveController;
    private final Drive driveSubsystem;
    private PIDController turnController;

    private static double kP = DriveConstants.autoAimKp;
    private static double kD = DriveConstants.autoAimKd;

    private double tunablekP = SmartDashboard.getNumber("Drive/AutoAim/kP", DriveConstants.autoAimKp);
    private double tunablekD = SmartDashboard.getNumber("Drive/AutoAim/kD", DriveConstants.autoAimKd);

    public AutoAim(CommandXboxController pDriveController, Drive pDrive) {
        this.driveController = pDriveController;
        this.driveSubsystem = pDrive;
        turnController = new PIDController(kP,0,kD);
        addRequirements(pDrive);
        instantiateTunables();
    }

    private void instantiateTunables() {
        SmartDashboard.putNumber("Drive/AutoAim/kP", DriveConstants.autoAimKp);
        SmartDashboard.putNumber("Drive/AutoAim/kD", DriveConstants.autoAimKd);
    }

    @Override
    public void initialize() {
        turnController = new PIDController(tunablekP,0,tunablekD);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        double currentAngle = currentPose.getRotation().getDegrees();
        
        Rotation2d goalRotation = turnFromHub(currentPose);
        double goalAngle = goalRotation.getDegrees();

        double calculatedOutput = turnController.calculate(currentAngle, goalAngle);

        SmartDashboard.putNumber("Drive/AutoAim/Goal Angle (DEG)", goalAngle);
        SmartDashboard.putNumber("Drive/AutoAim/Output", calculatedOutput);
        
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -calculatedOutput
        );
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    //Gets Hub pose according to alliance color
    public static Pose2d getHub() {
        return AllianceFlipUtil.apply(FieldConstants.kHubPose);
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    private static Rotation2d turnFromHub(Pose2d robotPose) {
        Pose2d hubCenter = getHub();
        Rotation2d angleFromhubCenter = Rotation2d.fromRadians(
                Math.atan2(hubCenter.getY() - robotPose.getY(), hubCenter.getX() - robotPose.getX()));

        Rotation2d finalAngle = angleFromhubCenter;
        // if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
        //     finalAngle = angleFromhubCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", finalAngle);
        return finalAngle;
    }
}
