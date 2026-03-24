package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
      new LoggedNetworkNumber("Shooter/Flywheel/kP", 0.01);
  private final LoggedNetworkNumber flyWheeltD =
      new LoggedNetworkNumber("Shooter/Flywheel/kD", 0.0);
  private final LoggedNetworkNumber flyWheeltV =
      new LoggedNetworkNumber("Shooter/Flywheel/kV", 1 / 6000);
  private final LoggedNetworkNumber flyWheeltS =
      new LoggedNetworkNumber("Shooter/Flywheel/kS", 0.15);

  private final LoggedNetworkNumber rollertP = new LoggedNetworkNumber("Shooter/Roller/kP", 0.01);
  private final LoggedNetworkNumber rollertD = new LoggedNetworkNumber("Shooter/Roller/kD", 0.0);
  private final LoggedNetworkNumber rollertV =
      new LoggedNetworkNumber("Shooter/Roller/kV", 1 / 6000);
  private final LoggedNetworkNumber rollertS = new LoggedNetworkNumber("Shooter/Roller/kS", 0.31);

  private final LoggedNetworkNumber allowedError =
      new LoggedNetworkNumber("Shooter/MaxMotion/Error", 100);
  private final LoggedNetworkNumber accel =
      new LoggedNetworkNumber("Shooter/MaxMotion/Acceleration", 3000);

  // Tuneable Setpoints
  private final LoggedNetworkNumber flywheelSetpoint =
      new LoggedNetworkNumber("Shooter/Flywheel/SetpointRPM", 4000);
  private final LoggedNetworkNumber rollerSetpoint =
      new LoggedNetworkNumber("Shooter/Roller/SetpointRPM", 3000);

  // Feedforward

  private final SimpleMotorFeedforward flywheelFF =
      new SimpleMotorFeedforward(flyWheeltV.get(), flyWheeltS.get());

  private final SimpleMotorFeedforward rollerFF =
      new SimpleMotorFeedforward(rollertV.get(), rollertS.get());

  public Turret() {

    // Flywheel config

    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    MAXMotionConfig speedConfig = new MAXMotionConfig();
    flywheelConfig.inverted(true);

    flywheelConfig.idleMode(IdleMode.kCoast);
    flywheelConfig.smartCurrentLimit(40);
    flywheelConfig.inverted(true);

    flywheelConfig.closedLoop.pid(flyWheeltP.get(), 0, flyWheeltD.get());
    flywheelConfig.closedLoop.feedForward.kV(flyWheeltV.get());
    flywheelConfig.closedLoop.feedForward.kS(flyWheeltS.get());
    flywheelConfig.closedLoop.maxMotion.apply(speedConfig);
    flyWheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    speedConfig.allowedProfileError(allowedError.get());
    speedConfig.maxAcceleration(accel.get());

    // Roller config

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig.inverted(false);
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(40);

    rollerConfig.closedLoop.pid(rollertP.get(), 0, rollertD.get());
    rollerConfig.closedLoop.maxMotion.apply(speedConfig);

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Flywheel control

  public FunctionalCommand setFlywheelRPM(double rpm) {
    double ffVolts = flywheelFF.calculate(rpm / -60.0);

    return new FunctionalCommand(
        () -> {
          flyWheelController.setSetpoint(
              rpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
        },
        () -> {},
        (interrupted) -> {
          flyWheelMotor.set(0.0);
        },
        () -> false);
  }

  public FunctionalCommand setRPMsTunable() {
    return new FunctionalCommand(
        () -> {
          double flywheelRPM = flywheelSetpoint.get();
          double rollerRPM = rollerSetpoint.get();

          flyWheelController.setSetpoint(
              flywheelRPM, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rollerRPM, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
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
          double rollerRPM = flywheelRPM - 325;

          flyWheelController.setSetpoint(
              flywheelRPM, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
          rollerController.setSetpoint(
              rollerRPM, SparkBase.ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
        },
        () -> {},
        (interrupted) -> {},
        () -> false);
  }

  public double getFlywheelRPM() {
    return flyWheelEncoder.getVelocity();
  }

  // Roller control

  public Command setRollerRPM(double rpm) {
    double ffVolts = rollerFF.calculate(rpm / 60.0);

    return this.run(
        () ->
            rollerController.setSetpoint(
                rpm, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts));
  }

  public double getRollerRPM() {
    return rollerEncoder.getVelocity();
  }

  public double getRollerVelocity() {
    return rollerEncoder.getVelocity();
  }

  public Command spinShooter(double wheelRPMs) {
    return this.runEnd(
        () -> {
          setFlywheelRPM(wheelRPMs);
          setRollerRPM(wheelRPMs);
        },
        () -> {
          setFlywheelRPM(0);
          setRollerRPM(0);
        });
  }

  public void spinShootervoid(double wheelRPMs) {
    this.runEnd(
        () -> {
          setFlywheelRPM(wheelRPMs);
          setRollerRPM(wheelRPMs);
        },
        () -> {
          setFlywheelRPM(0);
          setRollerRPM(0);
        });
  }

  public Command rawSpinShooter() {
    return this.runEnd(
        () -> {
          flyWheelMotor.setVoltage(12);
          rollerMotor.setVoltage(12);
          updateOdometry();
        },
        () -> {
          flyWheelMotor.set(0);
          rollerMotor.set(0);
        });
  }

  public Command stopShooter() {
    return this.run(
        () -> {
          flyWheelMotor.set(0);
          rollerMotor.set(0);
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

    double currentError = allowedError.get();
    double currentAccel = accel.get();

    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    MAXMotionConfig speedConfig = new MAXMotionConfig();

    if (flyWheelkP != flyWheeltP.get()
        || flyWheelkD != flyWheeltD.get()
        || flyWheelkV != flyWheeltV.get()
        || flyWheelkS != flyWheeltS.get()) {

      flywheelConfig.idleMode(IdleMode.kCoast);
      flywheelConfig.smartCurrentLimit(60);
      flywheelConfig.inverted(true);

      flywheelConfig.closedLoop.pid(flyWheeltP.get(), 0, flyWheeltD.get());

      flyWheelMotor.configure(
          flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

      rollerMotor.configure(
          rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      rollerMotor.configure(
          rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    if (currentError != allowedError.get() || currentAccel != accel.get()) {
      speedConfig.allowedProfileError(allowedError.get());
      speedConfig.maxAcceleration(accel.get());
    }
  }
}
