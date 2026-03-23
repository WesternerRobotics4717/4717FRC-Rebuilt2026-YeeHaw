package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  private final SparkFlex hoodMotor =
      new SparkFlex(ShooterConstants.hoodCanId, SparkFlex.MotorType.kBrushless);

  private PIDController hoodPID;
  private ArmFeedforward hoodFF;

  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  private double hoodtP = 0.00675;
  private double hoodtD = 0.0;
  private double hoodtG = 0.01;
  private double hoodSetpoint = 20;
  private double hoodDeadband = MathUtil.applyDeadband(getHoodAngle(), .1);

  public Hood() {

    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.inverted(false);
    hoodConfig.smartCurrentLimit(25);

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPID = new PIDController(hoodtP, 0, hoodtD);
    hoodFF = new ArmFeedforward(0, hoodtG, 0);

    instantiateTuneables();
  }

  public void instantiateTuneables() {
    SmartDashboard.putNumber("Shooter/Hood/kP", hoodtP);
    SmartDashboard.putNumber("Shooter/Hood/kD", hoodtD);
    SmartDashboard.putNumber("Shooter/Hood/kG", hoodtG);
    SmartDashboard.putNumber("Shooter/Hood/Setpoint", hoodSetpoint);
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
          double newSetpoint = SmartDashboard.getNumber("Shooter/Hood/Setpoint", hoodSetpoint);
          hoodPID.setSetpoint(newSetpoint);
          double outputPID = hoodPID.calculate(getHoodAngle());
          double outputFF = hoodFF.calculate(newSetpoint % ShooterConstants.conversionFactor, 0);
          hoodMotor.set(outputPID + outputFF);
          SmartDashboard.putNumber("Shooter/Hood/OutputPID", outputPID);
          SmartDashboard.putNumber("Shooter/Hood/OutputFF", outputFF);
        },
        (interrupted) -> {},
        () -> false);
  }

  public void periodic() {
    periodicTunables();
  }

  public void periodicTunables() {
    double currentHoodkP = hoodtP;
    double currentHoodkD = hoodtD;
    double currentHoodkG = hoodtG;
    SmartDashboard.putNumber("Shooter/Hood/Current Position", getHoodAngle());

    hoodtP = SmartDashboard.getNumber("Shooter/Hood/kP", hoodtP);
    hoodtD = SmartDashboard.getNumber("Shooter/Hood/kD", hoodtD);
    hoodtG = SmartDashboard.getNumber("Shooter/Hood/kG", hoodtG);

    if (currentHoodkP != hoodtP || currentHoodkD != hoodtD || currentHoodkG != hoodtG) {
      hoodPID = new PIDController(hoodtP, 0, hoodtD);
      hoodFF = new ArmFeedforward(0, hoodtG, 0);
    }
  }
}
