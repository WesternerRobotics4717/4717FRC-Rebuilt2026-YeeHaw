package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    addRequirements(turret, intake);
  }

  @Override
  public void initialize() {
    intake.moveArmDown();
    indexer.spinIndexer();
    intake.runIntake(5);
  }

  @Override
  public void execute() {
    Commands.repeatingSequence((intake.moveArmDown()));
  }
}
