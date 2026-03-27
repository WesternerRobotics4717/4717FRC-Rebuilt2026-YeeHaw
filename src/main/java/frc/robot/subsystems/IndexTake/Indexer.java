package frc.robot.subsystems.IndexTake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  // Motors
  private final SparkMax indexBottom =
      new SparkMax(IndexTakeConstants.indexerBottomCanId, MotorType.kBrushless);
  private final SparkFlex indexTop =
      new SparkFlex(IndexTakeConstants.indexerTopCanId, SparkFlex.MotorType.kBrushless);

  boolean hasStartedFiring = false;

  public Indexer() {
    SparkFlexConfig topConfig = new SparkFlexConfig();
    SparkMaxConfig bottomConfig = new SparkMaxConfig();

    topConfig.idleMode(IdleMode.kCoast);
    bottomConfig.idleMode(IdleMode.kCoast);
    topConfig.smartCurrentLimit(40);
    bottomConfig.smartCurrentLimit(40);

    indexBottom.configure(
        bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexTop.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command spinBottomIndexer(double voltage) {
    return this.run(() -> indexBottom.setVoltage(voltage));
  }

  public Command spinTopIndexer(double voltage) {
    return this.run(() -> indexTop.setVoltage(voltage));
  }

  public Command spinIndexer() {
    return this.runEnd(
        () -> {
          indexBottom.setVoltage(9);
          indexTop.setVoltage(-9);
        },
        () -> {
          indexBottom.setVoltage(0);
          indexTop.setVoltage(0);
        });
  }

  public void indexerVoid(double voltage) {
    this.runEnd(
        () -> {
          indexBottom.setVoltage(voltage);
          indexTop.setVoltage(-voltage);
        },
        () -> {
          indexBottom.setVoltage(0);
          indexTop.setVoltage(0);
        });
  }

  public Command spinIndexerOut() {
    return this.runEnd(
        () -> {
          indexBottom.setVoltage(-9);
          indexTop.setVoltage(9);
        },
        () -> {
          indexBottom.setVoltage(0);
          indexTop.setVoltage(0);
        });
  }

  public Command stopIndexer() {
    return this.run(
        () -> {
          spinTopIndexer(0);
          spinBottomIndexer(0);
        });
  }

  public void stopIndexerVoid() {
    this.run(
        () -> {
          spinTopIndexer(0);
          spinBottomIndexer(0);
        });
  }

  public FunctionalCommand runIndexer(double voltage) {
    return new FunctionalCommand(
        () -> {
          indexBottom.setVoltage(voltage);
          indexTop.set(-voltage * (100 / 12));
        },
        () -> {},
        (interrupted) -> {
          indexBottom.setVoltage(0);
          indexTop.setVoltage(0);
        },
        () -> false,
        this);
  }

  public FunctionalCommand fireFuel() {
    return new FunctionalCommand(
        () -> {
          hasStartedFiring = false;
        },
        () -> {
          boolean isHoodAtAngle = SmartDashboard.getBoolean("AutoAim/HoodAtSetpoint", false);
          boolean isFlywheelAtRPM = SmartDashboard.getBoolean("AutoAim/FlywheelsAtSetpoint", false);
          if (isHoodAtAngle && isFlywheelAtRPM && !hasStartedFiring) {
            hasStartedFiring = true;
            indexBottom.setVoltage(IndexTakeConstants.indexerShotVoltage);
            indexTop.set(-IndexTakeConstants.indexerShotVoltage * (100 / 12));
          }
        },
        (interrupted) -> {
          indexBottom.setVoltage(0);
          indexTop.setVoltage(0);
        },
        () -> false,
        this);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/TopOutput", indexTop.getAppliedOutput());
    Logger.recordOutput("Indexer/BottomOutput", indexBottom.getAppliedOutput());
  }
}
