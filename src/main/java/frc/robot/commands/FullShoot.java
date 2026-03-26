package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexTake.Indexer;
import frc.robot.subsystems.IndexTake.Intake;
import frc.robot.subsystems.Shooter.Turret;

public class FullShoot extends Command {

  private final Turret turret;
  private final Intake intake;
  private final Indexer indexer;
  private final Timer timer = new Timer();

  private boolean rollerGo = false;
  private double targetRPM;

  public FullShoot(Turret turret, Indexer indexer, Intake intake) {
    this.indexer = indexer;
    this.intake = intake;
    this.turret = turret;
    addRequirements(turret, intake, indexer);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("ShootingFuel");
    turret.setRPMs(targetRPM);
    rollerGo = false;
  }

  @Override
  public void execute() {
    if (!rollerGo && timer.hasElapsed(.5)) {
      indexer.indexerVoid(9);
      rollerGo = true;
    }

    // Commands.repeatingSequence((intake.setArmPosition()));
  }

  public void end(boolean interrupted) {
    indexer.indexerVoid(0);
    turret.stopShooter();
  }
}
