package frc.robot.subsystems.Shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {

  private final SparkFlex hoodMotor =
      new SparkFlex(ShooterConstants.hoodCanId, SparkFlex.MotorType.kBrushless);

  private PIDController hoodPID;
  private ArmFeedforward hoodFF;
  // 131.33953857421875
  private final AbsoluteEncoder hoodEncoder = hoodMotor.getAbsoluteEncoder();

  double hoodOffset = 0.36969376;

  private final LoggedNetworkNumber tuneablekP =
      new LoggedNetworkNumber("Tuning/Shooter/Hood/kP", 0.00835);
  private final LoggedNetworkNumber tuneablekD =
      new LoggedNetworkNumber("Tuning/Shooter/Hood/kD", 0.0);
  private final LoggedNetworkNumber tuneablekG =
      new LoggedNetworkNumber("Tuning/Shooter/Hood/kG", 0.0195);
  private final LoggedNetworkNumber hoodSetpoint =
      new LoggedNetworkNumber("Tuning/Shooter/Hood/Setpoint", 20);
  private final LoggedNetworkNumber hoodPosition =
      new LoggedNetworkNumber("Tuning/Shooter/Hood/Position", getHoodAngle());
  private final LoggedNetworkNumber hoodEncoderOffset =
      new LoggedNetworkNumber("Tuning/Shooter/Hood/Offset", hoodOffset);

  public Hood() {

    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.inverted(false);
    hoodConfig.smartCurrentLimit(25);
    hoodConfig.absoluteEncoder.positionConversionFactor(ShooterConstants.conversionFactor);
    hoodConfig.absoluteEncoder.zeroOffset(hoodOffset);
    hoodConfig.absoluteEncoder.inverted(true);
    hoodConfig.closedLoop.positionWrappingEnabled(true);

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPID = new PIDController(tuneablekP.get(), 0, tuneablekD.get());
    hoodPID.setTolerance(2);
    hoodFF = new ArmFeedforward(0, tuneablekG.get(), 0);
  }

  public double getHoodAngle() {

    if (hoodEncoder.getPosition() > 40) {
      return hoodEncoder.getPosition() - ShooterConstants.conversionFactor;
    } else return hoodEncoder.getPosition();
  }

  public Command rawMoveHood(double speed) {
    return this.runEnd(() -> hoodMotor.set(speed), () -> hoodMotor.set(0));
  }

  // public Command setHoodAngle(double angle) {
  //  return this.runOnce(() -> hoodEncoder
  // }

  public Command zeroHood() {
    return Commands.sequence(
        this.run(() -> hoodMotor.set(-.25)),
        Commands.waitSeconds(.25),
        this.run(() -> hoodMotor.stopMotor()));
  }

  public FunctionalCommand hoodPIDMove() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          double newSetpoint = (hoodSetpoint.get());
          hoodPID.setSetpoint(newSetpoint);
          double outputPID = hoodPID.calculate(getHoodAngle(), newSetpoint);
          double outputFF = hoodFF.calculate(Units.degreesToRadians(newSetpoint), 0);
          hoodMotor.set(outputPID + outputFF);
          SmartDashboard.putNumber("Shooter/Hood/OutputPID", outputPID);
          SmartDashboard.putNumber("Shooter/Hood/OutputFF", outputFF);
          SmartDashboard.putBoolean("AutoAim/HoodAtSetpoint", hoodPID.atSetpoint());
        },
        (interrupted) -> {},
        () -> false,
        this);
  }

  public FunctionalCommand hoodAutoAim(DoubleSupplier setpoint) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          hoodPID.setSetpoint(setpoint.getAsDouble());
          hoodPID.setPID(tuneablekP.get(), 0.0, tuneablekD.get());
          double outputPID = hoodPID.calculate(getHoodAngle(), setpoint.getAsDouble());
          double outputFF = hoodFF.calculate(Units.degreesToRadians(setpoint.getAsDouble()), 0);
          hoodMotor.set(outputPID + outputFF);
          SmartDashboard.putNumber("Shooter/Hood/OutputPID", outputPID);
          SmartDashboard.putNumber("Shooter/Hood/OutputFF", outputFF);
          SmartDashboard.putNumber("AutoAim/HoodSetpoint", setpoint.getAsDouble());
        },
        (interrupted) -> {
          zeroHood();
        },
        () -> false,
        this);
  }

  public FunctionalCommand hoodInputMove(double Setpoint) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          double newSetpoint = Setpoint;
          hoodPID.setSetpoint(newSetpoint);
          double outputPID = hoodPID.calculate(getHoodAngle());
          double outputFF = hoodFF.calculate(newSetpoint, 0);
          hoodMotor.set(outputPID + outputFF);
          SmartDashboard.putNumber("Shooter/Hood/OutputPID", outputPID);
          SmartDashboard.putNumber("Shooter/Hood/OutputFF", outputFF);
        },
        (interrupted) -> {},
        () -> false,
        this);
  }

  public void hoodVoidMove(double Setpoint) {
    new FunctionalCommand(
        () -> {},
        () -> {
          double newSetpoint = Setpoint;
          hoodPID.setSetpoint(newSetpoint);
          double outputPID = hoodPID.calculate(getHoodAngle());
          double outputFF = hoodFF.calculate(newSetpoint, 0);
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
    SmartDashboard.putBoolean("AutoAim/HoodAtSetpoint", hoodPID.atSetpoint());
    SmartDashboard.putNumber("Shooter/Hood/Output", hoodMotor.getAppliedOutput());
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
