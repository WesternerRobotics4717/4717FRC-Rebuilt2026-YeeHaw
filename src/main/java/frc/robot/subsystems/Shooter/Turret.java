package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

  // Motors
  // TODO: Add a ratio for rollerRPM to flyWheel RPM
  private final SparkFlex flyWheelMotor =
      new SparkFlex(ShooterConstants.flyWheelCanId, SparkFlex.MotorType.kBrushless);
  private final SparkFlex rollerMotor =
      new SparkFlex(ShooterConstants.rollerCanId, SparkFlex.MotorType.kBrushless);

  // Encoders

  private final RelativeEncoder flyWheelEncoder = flyWheelMotor.getEncoder();
  private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  // Controllers

  private final SparkClosedLoopController flyWheelController =
      flyWheelMotor.getClosedLoopController();

  private final SparkClosedLoopController rollerController = rollerMotor.getClosedLoopController();

  // PID

  private final LoggedNetworkNumber flyWheeltP =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel/kP", 0.002);
  private final LoggedNetworkNumber flyWheeltD =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel/kD", 0.0001);
  private final LoggedNetworkNumber flyWheeltV =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel/kV", .001845);
  private final LoggedNetworkNumber flyWheeltS =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel/kS", 0);
  private final LoggedNetworkNumber flyWheeltA =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel/kA", 0.001);

  private final LoggedNetworkNumber rollertP =
      new LoggedNetworkNumber("/Tuning/Shooter/Roller/kP", 0.0001);
  private final LoggedNetworkNumber rollertD =
      new LoggedNetworkNumber("/Tuning/Shooter/Roller/kD", 0.0);
  private final LoggedNetworkNumber rollertV =
      new LoggedNetworkNumber("/Tuning/Shooter/Roller/kV", 0.0018575);
  private final LoggedNetworkNumber rollertS =
      new LoggedNetworkNumber("/Tuning/Shooter/Roller/kS", 0.0);
  private final LoggedNetworkNumber rollertA =
      new LoggedNetworkNumber("/Tuning/Shooter/Roller/kA", 0.00001);

  // Tuneable Setpoints
  private final LoggedNetworkNumber flywheelSetpoint =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel/SetpointRPM", 4000);
  private final LoggedNetworkNumber rollerSetpoint =
      new LoggedNetworkNumber("/Tuning/Shooter/Roller/SetpointRPM", 3000);

  // Feedforward

  public Turret() {

    // Flywheel config

    SparkFlexConfig flywheelConfig = new SparkFlexConfig();

    flywheelConfig.idleMode(IdleMode.kCoast);
    flywheelConfig.smartCurrentLimit(40);
    flywheelConfig.inverted(true);

    flywheelConfig.closedLoop.pid(flyWheeltP.get(), 0, flyWheeltD.get());
    flywheelConfig.closedLoop.feedForward.kV(flyWheeltV.get());
    flywheelConfig.closedLoop.feedForward.kS(flyWheeltS.get());
    flywheelConfig.closedLoop.feedForward.kA(flyWheeltA.get());
    flywheelConfig.closedLoop.allowedClosedLoopError(0.0, ClosedLoopSlot.kSlot0);
    flyWheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Roller config

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig.inverted(false);
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(40);
    rollerConfig.closedLoop.feedForward.kV(rollertV.get());
    rollerConfig.closedLoop.feedForward.kS(rollertS.get());
    rollerConfig.closedLoop.feedForward.kA(rollertA.get());
    rollerConfig.closedLoop.allowedClosedLoopError(0.0, ClosedLoopSlot.kSlot0);

    rollerConfig.closedLoop.pid(rollertP.get(), 0, rollertD.get());

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Flywheel control

  public FunctionalCommand setFlywheelRPM(double rpm) {

    return new FunctionalCommand(
        () -> {
          flyWheelController.setSetpoint(
              rpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        },
        () -> {},
        (interrupted) -> {
          flyWheelMotor.set(0.0);
        },
        () -> false);
  }

  public FunctionalCommand setAutoRPM(DoubleSupplier rpm) {

    return new FunctionalCommand(
        () -> {
          flyWheelController.setSetpoint(
              rpm.getAsDouble(), SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rpm.getAsDouble(), SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        },
        () -> {
          flyWheelController.setSetpoint(
              rpm.getAsDouble(), SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rpm.getAsDouble(), SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          SmartDashboard.putNumber("AutoAim/Goal Flywheel RPM", rpm.getAsDouble());
          SmartDashboard.putNumber("AutoAim/Goal Roller RPM", rpm.getAsDouble());
          SmartDashboard.putBoolean(
              "AutoAim/FlywheelsAtSetpoint",
              Math.abs(flyWheelEncoder.getVelocity() - rpm.getAsDouble())
                  < ShooterConstants.allowedError);
          SmartDashboard.putBoolean(
              "AutoAim/RollersAtSetpoint",
              Math.abs(rollerEncoder.getVelocity() - rpm.getAsDouble())
                  < ShooterConstants.allowedError);
        },
        (interrupted) -> {
          flyWheelMotor.set(0.0);
          rollerMotor.set(0);
        },
        () -> false);
  }

  public FunctionalCommand setRPMsTunable() {
    return new FunctionalCommand(
        () -> {
          double flywheelRPM = flywheelSetpoint.get();
          double rollerRPM = rollerSetpoint.get();

          flyWheelController.setSetpoint(
              flywheelRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rollerRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        },
        () -> {
          updateOdometry();
        },
        (interrupted) -> {
          flyWheelMotor.set(0.0);
          rollerMotor.set(0.0);
        },
        () -> false);
  }

  public FunctionalCommand setRPMs(double targetRPM) {
    return new FunctionalCommand(
        () -> {
          double flywheelRPM = targetRPM;
          double rollerRPM = targetRPM;

          flyWheelController.setSetpoint(
              flywheelRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rollerRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        },
        () -> {},
        (interrupted) -> {},
        () -> false,
        this);
  }

  public double getFlywheelRPM() {
    return flyWheelEncoder.getVelocity();
  }

  public void spinShooterVoid(double targetRPM) {
    new FunctionalCommand(
        () -> {
          double flywheelRPM = targetRPM;
          double rollerRPM = targetRPM;

          flyWheelController.setSetpoint(
              flywheelRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rollerRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        },
        () -> {
          updateOdometry();
        },
        (interrupted) -> {
          flyWheelMotor.set(0.0);
          rollerMotor.set(0.0);
        },
        () -> false);
  }

  // Roller control

  public Command setRollerRPM(double rpm) {

    return this.run(
        () ->
            rollerController.setSetpoint(
                rpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0));
  }

  public double getRollerRPM() {
    return rollerEncoder.getVelocity();
  }

  public double getRollerVelocity() {
    return rollerEncoder.getVelocity();
  }
  ;

  public Command rawSpinShooter(double voltage) {
    return this.runEnd(
        () -> {
          flyWheelMotor.setVoltage(voltage);
          rollerMotor.setVoltage(voltage);
          updateOdometry();
        },
        () -> {
          flyWheelMotor.set(0);
          rollerMotor.set(0);
        });
  }

  public Command slowShooter() {
    return this.runOnce(
        () -> {
          flyWheelController.setSetpoint(
              ShooterConstants.restingShooterRPM,
              SparkBase.ControlType.kVelocity,
              ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              ShooterConstants.restingShooterRPM,
              SparkBase.ControlType.kVelocity,
              ClosedLoopSlot.kSlot0);
        });
  }

  public Command fullStopShooter() {
    return this.runOnce(
        () -> {
          flyWheelMotor.set(0);
          rollerMotor.set(0);
          flyWheelController.setSetpoint(
              ShooterConstants.restingShooterRPM,
              SparkBase.ControlType.kVelocity,
              ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              ShooterConstants.restingShooterRPM,
              SparkBase.ControlType.kVelocity,
              ClosedLoopSlot.kSlot0);
        });
  }

  public void stopShooterVoid() {
    this.run(
        () -> {
          flyWheelMotor.set(0);
          rollerMotor.set(0);
          flyWheelController.setSetpoint(0, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(0, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        });
  }

  public void periodic() {
    updateValues();
    updateOdometry();
  }

  public void updateOdometry() {
    final double flyWheelRPM = flyWheelMotor.getEncoder().getVelocity();
    final double flyWheelDutyCycle = flyWheelMotor.getAppliedOutput();
    final double rollerRPM = rollerMotor.getEncoder().getVelocity();
    final double rollerDutyCycle = rollerMotor.getAppliedOutput();

    Logger.recordOutput("Shooter/Flywheel/RPM", flyWheelRPM);
    Logger.recordOutput("Shooter/Flywheel/DutyCycle Output", flyWheelDutyCycle);
    SmartDashboard.putBoolean("AutoAim/FlywheelsAtSetpoint", flyWheelController.isAtSetpoint());
    Logger.recordOutput("Shooter/Roller/RPM", rollerRPM);
    Logger.recordOutput("Shooter/Roller/DutyCycle Output", rollerDutyCycle);
  }

  public void updateValues() {
    double flyWheelkP = flyWheeltP.get();
    double flyWheelkD = flyWheeltD.get();
    double flyWheelkV = flyWheeltV.get();
    double flyWheelkS = flyWheeltS.get();

    double rollerkP = rollertP.get();
    double rollerkD = rollertD.get();
    double rollerkV = rollertV.get();
    double rollerkS = rollertS.get();

    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkFlexConfig rollerConfig = new SparkFlexConfig();

    if (flyWheelkP != flyWheeltP.get()
        || flyWheelkD != flyWheeltD.get()
        || flyWheelkV != flyWheeltV.get()
        || flyWheelkS != flyWheeltS.get()) {

      flywheelConfig.idleMode(IdleMode.kCoast);
      flywheelConfig.smartCurrentLimit(40);
      flywheelConfig.inverted(true);

      flywheelConfig.closedLoop.pid(flyWheelkP, 0, flyWheelkD);
      flywheelConfig.closedLoop.feedForward.kV(flyWheelkV);
      flywheelConfig.closedLoop.feedForward.kS(flyWheelkS);

      flyWheelMotor.configure(
          flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    if (rollerkP != rollertP.get()
        || rollerkD != rollertD.get()
        || rollerkV != rollertV.get()
        || rollerkS != rollertS.get()) {
      rollerConfig.idleMode(IdleMode.kCoast);
      rollerConfig.smartCurrentLimit(45);

      rollerConfig.closedLoop.pid(rollertP.get(), 0, rollertD.get());
      rollerConfig.closedLoop.feedForward.kV(rollertV.get());
      rollerConfig.closedLoop.feedForward.kS(rollertS.get());

      rollerMotor.configure(
          rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
}
