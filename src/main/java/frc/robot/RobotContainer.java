// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FullShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IndexTake.Indexer;
import frc.robot.subsystems.IndexTake.Intake;
import frc.robot.subsystems.Shooter.Hood;
import frc.robot.subsystems.Shooter.ShotMap;
import frc.robot.subsystems.Shooter.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.LocalizationSystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  public final Turret shooter;
  public final Intake intake;
  public final Indexer indexer;
  public final Hood hood;
  public final FullShoot shootFuel;
  public final AutoAim autoAim;
  public final ShotMap shotMap;
  public final LocalizationSystem questNav;

  // public final AutoAim aimRobot;

  // Controller
  private final CommandXboxController swerver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // TODO: continue working on autonomous. Add control switches, for solo and duo.
  // TODO: Figure out why Advantage kit isnt logging

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardChooser<Command> driverControls;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    shooter = new Turret();
    intake = new Intake();
    indexer = new Indexer();
    hood = new Hood();
    shotMap = new ShotMap();
    questNav = new LocalizationSystem();

    // Register Commands
    shootFuel = new FullShoot(shooter, indexer, intake);
    autoAim = new AutoAim(drive, shotMap);

    NamedCommands.registerCommand("runWheel", (new AutoAim(drive, shotMap)).withTimeout(6));

    NamedCommands.registerCommand(
        "intakingDown",
        Commands.parallel(intake.rawMoveIntake(-8).withTimeout(.7), intake.runIntake(6))
            .withTimeout((1)));

    NamedCommands.registerCommand(
        "intakingUp",
        Commands.parallel(intake.rawMoveIntake(5).withTimeout(.5), intake.runIntake(0)));

    NamedCommands.registerCommand("intakeGo", intake.runIntake(6));

    // Event Triggers
    new EventTrigger("dropIntake").onTrue(intake.rawMoveIntake(-8).withTimeout(.75));
    new EventTrigger("runIntake")
        .whileTrue(intake.runIntake(7.5).alongWith(indexer.shuffleBottomIndexer()));
    new EventTrigger("shittyScoreClose")
        .whileTrue(
            Commands.parallel(
                shooter.setRPMs(3000),
                hood.hoodInputMove(9.5),
                Commands.waitSeconds(.5).andThen(indexer.spinIndexer()),
                intake.runIntake(6),
                intake.ezUpDown().withTimeout(1.25)))
        .onFalse(intake.rawMoveIntake(-8).withTimeout(.5));
    new EventTrigger("shittyScoreFar")
        .whileTrue(
            Commands.parallel(
                shooter.setRPMs(3800),
                hood.hoodInputMove(17),
                Commands.waitSeconds(.5).andThen(indexer.spinIndexer()),
                intake.runIntake(6),
                intake.ezUpDown().withTimeout(1.25)))
        .onFalse(intake.rawMoveIntake(-8).withTimeout(.5));
    new EventTrigger("autoFuel")
        .whileTrue(
            Commands.parallel(
                shooter.setAutoRPM(() -> shotMap.getRPM()),
                hood.hoodAutoAim(() -> shotMap.getAngle()),
                indexer.runIndexer(7),
                intake.runIntake(5),
                intake.armUpDown()))
        .onFalse(
            Commands.parallel(
                shooter.slowShooter(),
                hood.zeroHood(),
                indexer.stopIndexer(),
                intake.stopIntake()));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // driverControls = new LoggedDashboardChooser<>("DriverSelection");

    // Set up SysId routines
    /*
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */

    // Look at event trigger versus named commands

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -swerver.getLeftY(),
            () -> -swerver.getLeftX(),
            () -> -swerver.getRightX()));

    // Swerver Commands
    // Lock to 0° when Y button is held
    swerver
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -swerver.getLeftY(),
                () -> -swerver.getLeftX(),
                () -> Rotation2d.kZero));

    // Reset gyro to 0° when B button is pressed
    swerver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    swerver
        .a()
        .toggleOnTrue(Commands.parallel(intake.runIntake(7.5), indexer.shuffleBottomIndexer()));

    swerver.povUp().whileTrue(intake.rawMoveIntake(5));
    swerver.povDown().whileTrue(intake.rawMoveIntake(-5));

    swerver.leftBumper().whileTrue(Commands.parallel(indexer.runIndexer(-8), intake.runIntake(-4)));

    operator
        .a()
        .toggleOnTrue(Commands.parallel(intake.runIntake(7.5), indexer.shuffleBottomIndexer()));

    operator.povUp().whileTrue(intake.rawMoveIntake(5));
    operator.povDown().whileTrue(intake.rawMoveIntake(-5));

    operator.back().whileTrue(hood.accZeroHood());

    operator
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                shooter.rawSpinShooter(-6), indexer.runIndexer(-8), intake.runIntake(-4)));
    operator.leftBumper().whileTrue(indexer.runIndexer(-5));
    operator
        .povLeft()
        .whileTrue(
            Commands.parallel(
                shooter.setRPMs(3400.0),
                hood.hoodInputMove(10 + 9),
                Commands.waitSeconds(.25).andThen(indexer.spinIndexer()),
                intake.ezUpDown()))
        .onFalse(hood.accZeroHood().alongWith(shooter.slowShooter()));
    operator
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                new AutoAim(drive, shotMap),
                shooter.setAutoRPM(() -> shotMap.getRPM()),
                hood.hoodAutoAim(() -> shotMap.getAngle()),
                indexer.fireFuel(),
                intake.runIntake(5),
                intake.armUpDown()))
        .onFalse(hood.accZeroHood());
    operator
        .povRight()
        .whileTrue(
            Commands.parallel(
                shooter.setRPMs(2750.0),
                hood.hoodInputMove(6 + 9),
                Commands.waitSeconds(.25).andThen(indexer.spinIndexer()),
                intake.ezUpDown()))
        .onFalse(hood.accZeroHood().alongWith(shooter.slowShooter()));

    operator.b().toggleOnTrue(shooter.slowShooter());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
