package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexTake.Indexer;
import frc.robot.subsystems.IndexTake.Intake;
import frc.robot.subsystems.Shooter.Turret;

public class FullShoot extends Command {

  private final Turret turret;
  private final Intake intake;
  private final Indexer indexer;

  public FullShoot(Turret turret, Intake intake, Indexer indexer) {
    this.turret = turret;
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(turret, intake, indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println("shootingFuel");
    indexer.indexerRAW(6);

    // Commands.repeatingSequence((intake.moveArmDown()));
  }

  public void end(boolean interrupted) {
    indexer.indexerRAW(0);
  }
}
