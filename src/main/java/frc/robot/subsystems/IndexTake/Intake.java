package frc.robot.subsystems.IndexTake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  public static final CANBus kCANBus = new CANBus("CTRE Devs");
  // TODO: Need to get the encoder offset for the arm encoder
  // Those Who Declare
  private final SplineEncoder armSplineEncoder =
      new SplineEncoder(IndexTakeConstants.splineEncoderId);
  private final TalonFX armMotor = new TalonFX(IndexTakeConstants.intakeMoveCanId, kCANBus);
  private final TalonFX spinMotor = new TalonFX(IndexTakeConstants.intakeSpinCanId, kCANBus);
  private final TalonFX spinFollower =
      new TalonFX(IndexTakeConstants.intakeSpinFollowerCanId, kCANBus);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final TalonFXConfiguration armConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration spinConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration spinConfig2 = new TalonFXConfiguration();

  private final DetachedEncoderConfig encoderConfig = new DetachedEncoderConfig();

  private double armOffset = 0.67449456; // needs to be tuned

  boolean hasStartedFiring;

  // Variables

  // Offsets
  private final LoggedNetworkNumber tunablekP =
      new LoggedNetworkNumber("/Tuning/Intake/Pivot/kP", 2.0);
  private final LoggedNetworkNumber tunablekD =
      new LoggedNetworkNumber("/Tuning/Intake/Pivot/kD", 0.001);
  private final LoggedNetworkNumber tunablekG =
      new LoggedNetworkNumber("/Tuning/Intake/Pivot/kG", 0.9575);
  private final LoggedNetworkNumber intakeSetpoint =
      new LoggedNetworkNumber("/Tuning/Intake/Pivot/Setpoint", .2);

  private double lastkP = 0.0;
  private double lastkD = 0.0;
  private double lastkG = 0.0;

  public Intake() {
    armConfig.Feedback.RotorToSensorRatio = IndexTakeConstants.intakeConversionFactorRatio;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.Feedback.FeedbackRemoteSensorID = 44;

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.kG = tunablekG.get();
    armConfig.Slot0.kS = .5425;
    armConfig.Slot0.kP = tunablekP.get();
    armConfig.Slot0.kD = tunablekD.get();
    armConfig.MotionMagic.MotionMagicAcceleration = 100;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
    armConfig.MotionMagic.MotionMagicJerk = 500;
    armConfig.CurrentLimits.withStatorCurrentLimit(35);
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    encoderConfig.inverted(false);
    encoderConfig.dutyCycleOffset(armOffset);

    armSplineEncoder.configure(encoderConfig, ResetMode.kResetSafeParameters);

    spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spinConfig.CurrentLimits.withStatorCurrentLimit(50);
    spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    spinConfig2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spinConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    spinConfig2.CurrentLimits.withStatorCurrentLimit(50);
    spinConfig2.CurrentLimits.StatorCurrentLimitEnable = true;
    armMotor.getConfigurator().apply(armConfig);
    spinMotor.getConfigurator().apply(spinConfig);
    spinFollower.getConfigurator().apply(spinConfig2);

    armMotor.setPosition(armSplineEncoder.getPosition());
  }

  public Command setArmOffset() {
    return Commands.run(() -> armMotor.setPosition(armOffset));
  }

  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public Command setTunableArmPosition() {
    return this.run(
        () -> armMotor.setControl(motionMagicRequest.withPosition(intakeSetpoint.getAsDouble())));
  }

  public Command setArmPosition(double targetAngle) {
    double targetRot = targetAngle / 360;
    return this.run(() -> armMotor.setControl(motionMagicRequest.withPosition(targetRot)));
  }

  public FunctionalCommand smallTrashCompact(double voltage) {
    return new FunctionalCommand(
        () -> {
          hasStartedFiring = false;
        },
        () -> {
          boolean isHoodAtAngle = SmartDashboard.getBoolean("AutoAim/HoodAtSetpoint", false);
          boolean isFlywheelAtRPM = SmartDashboard.getBoolean("AutoAim/FlywheelsAtSetpoint", false);
          if (isHoodAtAngle && isFlywheelAtRPM && !hasStartedFiring) {
            armMotor.setVoltage(voltage);
            hasStartedFiring = true;
          }
          if ((((armSplineEncoder.getPosition() * 360) > 50) && voltage >= 0)
              || ((((armSplineEncoder.getPosition() * 360) < 0) && voltage <= 0)))
            armMotor.setVoltage(0.0);
        },
        (interrupted) -> {
          armMotor.setVoltage(0);
        },
        () ->
            (((armSplineEncoder.getPosition() * 360) > 50) && voltage >= 0)
                || (((armSplineEncoder.getPosition() * 360) < 0) && voltage <= 0));
  }

  public Command armUpDown() {
    return (Commands.repeatingSequence(
        smallTrashCompact(.85),
        Commands.waitSeconds(.25),
        smallTrashCompact(-1),
        Commands.waitSeconds(.25)));
  }

  public Command intakeArmStop() {
    return this.runOnce(() -> armMotor.setControl(new VoltageOut(0)));
  }

  // public Command moveArmToPosition() {
  // return this.run(() -> setArmPosition(newArmPosition));}

  public FunctionalCommand runIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          spinMotor.setControl(new VoltageOut(voltage));
          spinFollower.setControl(new VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          spinMotor.setControl(new VoltageOut(0));
          spinFollower.setControl(new VoltageOut(0));
        },
        () -> false);
  }

  public FunctionalCommand rawMoveIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          armMotor.setControl(new VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          armMotor.setControl(new VoltageOut(0));
        },
        () -> false);
  }

  public void runIntakeVoid(double voltage) {
    new FunctionalCommand(
        () -> {
          spinMotor.setControl(new VoltageOut(voltage));
          spinFollower.setControl(new VoltageOut(voltage));
        },
        () -> {},
        (interrupted) -> {
          spinMotor.setControl(new VoltageOut(0));
          spinFollower.setControl(new VoltageOut(0));
        },
        () -> false);
  }

  public void stopIntakeVoid() {
    spinMotor.setVoltage(0);
    spinFollower.setVoltage(0);
  }

  public Command stopIntake() {
    return this.run(() -> runIntake(0));
  }

  @Override
  public void periodic() {
    updateValues();
    Logger.recordOutput("Intake/Pivot/AbsEncoderRot", armSplineEncoder.getPosition());
    Logger.recordOutput("Intake/Pivot/AbsEncoderDeg", armSplineEncoder.getPosition() * 360);
  }

  public void updateValues() {
    double kP = tunablekP.get();
    double kD = tunablekD.get();
    double kG = tunablekG.get();

    if (kP != lastkP || kD != lastkD) {
      lastkP = kP;
      lastkD = kD;
      armMotor.getConfigurator().apply(armConfig);
    }

    if (kG != lastkG) {
      lastkG = kG;
      armConfig.Slot0.kG = kG;
      armMotor.getConfigurator().apply(armConfig);
    }
  }
}
