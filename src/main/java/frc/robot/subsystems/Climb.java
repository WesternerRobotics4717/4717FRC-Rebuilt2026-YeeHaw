package frc.robot.subsystems;

/*
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveConstants;

public class Climb extends SubsystemBase {

  // Motors

  private final SparkMax climbMotor = new SparkMax(DriveConstants.climbCanId, MotorType.kBrushless);

  public Climb() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    climbConfig.idleMode(IdleMode.kBrake);

    climbMotor.configure(
        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command climbUp() {
    return this.runEnd(() -> climbMotor.setVoltage(4), () -> climbMotor.setVoltage(0));
  }

  public Command climbDrop() {
    return this.runEnd(() -> climbMotor.setVoltage(-4), () -> climbMotor.setVoltage(0));
  }
}
  */
