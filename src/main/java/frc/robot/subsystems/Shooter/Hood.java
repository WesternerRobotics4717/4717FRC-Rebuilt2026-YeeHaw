package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {

  private final SparkFlex hoodMotor =
      new SparkFlex(ShooterConstants.hoodCanId, SparkFlex.MotorType.kBrushless);

  private PIDController hoodPID;
  private ArmFeedforward hoodFF;

  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  private final LoggedNetworkNumber tuneablekP =
      new LoggedNetworkNumber("Shooter/Hood/kP", 0.00835);
  private final LoggedNetworkNumber tuneablekD = new LoggedNetworkNumber("Shooter/Hood/kD", 0.0);
  private final LoggedNetworkNumber tuneablekG = new LoggedNetworkNumber("Shooter/Hood/kG", 0.0195);
  private final LoggedNetworkNumber hoodSetpoint =
      new LoggedNetworkNumber("Shooter/Hood/Setpoint", 20);
  private final LoggedNetworkNumber hoodPosition =
      new LoggedNetworkNumber("Shooter/Hood/Position", getHoodAngle());

  // TODO: Hood Conversion Factor, better, check if you need the if != and || is needed
  public Hood() {

    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.inverted(false);
    hoodConfig.smartCurrentLimit(25);

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPID = new PIDController(tuneablekP.get(), 0, tuneablekD.get());
    hoodFF = new ArmFeedforward(0, tuneablekG.get(), 0);
  }

  public double getHoodAngle() {
    return hoodEncoder.getPosition() * ShooterConstants.conversionFactor;
  }

  public Command rawMoveHood(double speed) {
    return this.runEnd(() -> hoodMotor.set(speed), () -> hoodMotor.set(0));
  }

  public Command setHoodAngle(double angle) {
    return this.runOnce(() -> hoodEncoder.setPosition(angle));
  }

  public Command zeroHood() {
    return Commands.sequence(
        this.run(() -> hoodMotor.set(-.25)),
        Commands.waitSeconds(.25),
        this.run(() -> hoodMotor.set(0)),
        setHoodAngle(0));
  }

  public FunctionalCommand hoodPIDMove() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          double newSetpoint = hoodSetpoint.get();
          hoodPID.setSetpoint(newSetpoint);
          double outputPID = hoodPID.calculate(getHoodAngle());
          double outputFF = hoodFF.calculate(newSetpoint % ShooterConstants.conversionFactor, 0);
          hoodMotor.set(outputPID + outputFF);
          SmartDashboard.putNumber("Shooter/Hood/OutputPID", outputPID);
          SmartDashboard.putNumber("Shooter/Hood/OutputFF", outputFF);
        },
        (interrupted) -> {},
        () -> false,
        this);
  }

  public void periodic() {
    periodicTunables();
    hoodPosition.set(getHoodAngle());
  }

  public void periodicTunables() {
    double kP = tuneablekP.get();
    double kD = tuneablekD.get();
    double kG = tuneablekG.get();

    if (kP != tuneablekP.get() || kD != tuneablekD.get() || kG != tuneablekG.get()) {
      hoodPID = new PIDController(kP, 0, kD);
      hoodFF = new ArmFeedforward(0, kG, 0);
    }
  }
}
