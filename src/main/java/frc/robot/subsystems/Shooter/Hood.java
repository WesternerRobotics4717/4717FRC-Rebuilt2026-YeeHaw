package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  private final SparkFlex hoodMotor =
      new SparkFlex(ShooterConstants.hoodCanId, SparkFlex.MotorType.kBrushless);

  private final PIDController hoodPID;

  private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  private final SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();

  private double hoodtP = 0.008;
  private double hoodtD = 0.0;
  private double hoodtG = 0.01;
  private double hoodFF = 0.0;

  public Hood() {

    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.inverted(false);

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPID = new PIDController(hoodtP, 0, hoodtD);

    instantiateTuneables();
  }

  public void instantiateTuneables() {
    SmartDashboard.putNumber("Shooter/Hood/kP", hoodtP);
    SmartDashboard.putNumber("Shooter/Hood/kD", hoodtD);
    SmartDashboard.putNumber("Shooter/Hood/kG", hoodtG);
  }

  public Command setHoodAngle(double angle) {
    return this.run(() -> hoodController.setSetpoint(angle, SparkBase.ControlType.kPosition));
  }

  public double getHoodAngle() {
    return hoodEncoder.getPosition() * ShooterConstants.conversionFactor;
  }

  public Command rawMoveHood(double speed) {
    return this.runEnd(() -> hoodMotor.set(speed), () -> hoodMotor.set(speed));
  }

  public FunctionalCommand hoodPIDMove(double setpoint) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          hoodPID.setSetpoint(setpoint);
          double output = hoodPID.calculate(getHoodAngle());
          hoodMotor.set(output + hoodFF);
          SmartDashboard.putNumber("Shooter/Hood/Output", output);
        },
        (interrupted) -> {
          hoodMotor.set(hoodFF);
        },
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

    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    if (currentHoodkP != hoodtP || currentHoodkD != hoodtD || currentHoodkG != hoodtG) {
      hoodPID.setPID(hoodtP, 0.0, hoodtD);

      hoodFF = hoodtG;

      hoodMotor.configure(
          hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}
