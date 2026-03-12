package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.DeviceIDs.driveIDs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DeviceIDs.driveIDs;





public class ModuleIOSpark implements ModuleIO {

  private final TalonFX driveMotor;
  private final SparkMax turnMotor;
  private final CANcoder cancoder;

  private final PIDController turnPID;

  private final Rotation2d zeroRotation;

  public ModuleIOSpark(int module) {

    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backRightZeroRotation;
          case 3 -> backLeftZeroRotation;
          default -> Rotation2d.kZero;
        };

    driveMotor =
        new TalonFX(
            switch (module) {
              case 0 -> driveIDs.frontLeftDriveCanId;
              case 1 -> driveIDs.frontRightDriveCanId;
              case 2 -> driveIDs.backRightDriveCanId;
              case 3 -> driveIDs.backLeftDriveCanId;
              default -> 0;
            });

    turnMotor =
        new SparkMax(
            switch (module) {
              case 0 -> driveIDs.frontLeftTurnCanId;
              case 1 -> driveIDs.frontRightTurnCanId;
              case 2 -> driveIDs.backRightTurnCanId;
              case 3 -> driveIDs.backLeftTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);

    cancoder =
        new CANcoder(
            switch (module) {
              case 0 -> driveIDs.frontLeftCANcoderCanId;
              case 1 -> driveIDs.frontRightCANcoderCanId;
              case 2 -> driveIDs.backRightCANcoderCanId;
              case 3 -> driveIDs.backLeftCANcoderCanId;
              default -> 0;
            });

    turnPID = new PIDController(turnKp, 0.0, turnKd);
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = DriveConstants.driveMotorReduction;

    config.Slot0.kP = DriveConstants.driveKp;
    config.Slot0.kV = DriveConstants.driveKv;
    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());

    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());

    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();

    inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();

    double turnRadians = Units.rotationsToRadians(cancoder.getAbsolutePosition().getValueAsDouble());

    inputs.turnPosition = new Rotation2d(turnRadians).minus(zeroRotation);

    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(cancoder.getVelocity().getValueAsDouble());

    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();

    inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double rps = velocityRadPerSec / (2 * Math.PI);
    driveMotor.setControl(new VelocityVoltage(rps));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {

    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), -Math.PI, Math.PI);

    double measurement =
        Units.rotationsToRadians(cancoder.getAbsolutePosition().getValueAsDouble());

    double output = turnPID.calculate(measurement, setpoint);

    turnMotor.setVoltage(output);
  }

  @Override
  public void setDrivePID(double driveKp, double driveKd, double driveKv, double driveKs) {}

  @Override
  public void setAzimuthPID(double turnKp, double turnKi, double turnKd) {
    turnPID.setPID(turnKp, turnKi, turnKd);
  }

  @Override
  public double getPIDOutput() {
    double measurement =
        Units.rotationsToRadians(cancoder.getAbsolutePosition().getValueAsDouble());

    return turnPID.calculate(measurement);
  }
}
