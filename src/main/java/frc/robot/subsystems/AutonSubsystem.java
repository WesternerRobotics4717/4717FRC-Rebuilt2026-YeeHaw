package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonSubsystem {
  private final Drive mDrive;
  private final Intake mIntake;
  private final Indexer mIndexer;
  private final Shooter mShooter;

  private final LoggedDashboardChooser<Command> autoChooser;

  public AutonSubsystem(Drive pDrive, Intake pIntake, Indexer pIndexer, Shooter pShooter) {
    mDrive = pDrive;
    mIntake = pIntake;
    mIndexer = pIndexer;
    mShooter = pShooter;

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand(() -> mDrive.resetGyro()));
    SmartDashboard.putData(autoChooser.getSendableChooser());
  }

  public Command getChosenAuton() {
    return autoChooser.get();
  }
}
