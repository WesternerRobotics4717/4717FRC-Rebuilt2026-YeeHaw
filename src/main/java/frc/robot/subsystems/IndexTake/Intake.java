package frc.robot.subsystems.IndexTake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // Those Who Declare

  private final SparkFlex armMoveMotor =
      new SparkFlex(IndexTakeConstants.intakeMoveCanId, MotorType.kBrushless);
  private final SparkFlex spinIntakeMotor =
      new SparkFlex(IndexTakeConstants.intakeSpinCanId, MotorType.kBrushless);

  private final RelativeEncoder relativeMoveEncoder = armMoveMotor.getEncoder();

  private final SparkClosedLoopController armMovePID = armMoveMotor.getClosedLoopController();

  // Constants (untuned)
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  private static double kG = 0;
  private static double kS = 0;
  private static double kMinOutput = -1;
  private static double kMaxOutput = 1;

  private final ArmFeedforward armFeedForward = new ArmFeedforward(kS, kG, 0);
  private final PIDController armController = new PIDController(kP, kI, kD);

  // Variables CUz  (need to be done, i dont rememb er them)
  private static final double arm_Position_Conversion_Factor = 60;
  private static final double arm_Velocity_Conversion_Factor = 4;
  private static final double MAX_Velocity = 120;
  private static final double MAX_Acceleration = 250;
  private static final double MAX_LoopError = 1.5;

  // Offsets
  private static double startPosition = 0;

  private double tunablekP = 0.0;
  private double tunablekI = 0.0;
  private double tunablekD = 0.0;
  private double tunablekG = 0.0;
  private double tunableMaxAccel = 0.0;
  private double tunableMaxVelocity = 0.0;

  public Intake() {
    SparkFlexConfig moveConfig = new SparkFlexConfig();
    SparkFlexConfig spinConfig = new SparkFlexConfig();

    moveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    moveConfig.smartCurrentLimit(40); // No clue on this guy

    moveConfig.encoder.positionConversionFactor(arm_Position_Conversion_Factor);

    // moveConfig.closedLoop.maxMotion
    //         .cruiseVelocity(MAX_Velocity)
    //         .maxAcceleration(MAX_Acceleration)
    //         .allowedProfileError(MAX_LoopError);

    // moveConfig.closedLoop
    //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //         .pid(kP, kI, kD)
    //         .outputRange(kMinOutput, kMaxOutput); //Max Fraction of Output

    armMoveMotor.configure(
        moveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    spinConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    spinConfig.smartCurrentLimit(40);
    spinConfig.encoder.velocityConversionFactor(arm_Velocity_Conversion_Factor);
    spinConfig.inverted(true);

    spinIntakeMotor.configure(
        spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    instantiateTunables();
  }

  private void instantiateTunables() {
    SmartDashboard.putNumber("Intake/Pivot/kP", kP);
    SmartDashboard.putNumber("Intake/Pivot/kI", 0);
    SmartDashboard.putNumber("Intake/Pivot/kD", kP);
    SmartDashboard.putNumber("Intake/Pivot/kG", kG);
    // SmartDashboard.putNumber("Intake/Pivot/Max Velocity", kP);
    // SmartDashboard.putNumber("Intake/Pivot/Max Acceleration", kP)

  }

  public double getArmPositionDegrees() {
    return relativeMoveEncoder.getPosition() * IndexTakeConstants.conversionFactor;
  }

  public Command setArmPosition(double targetDegrees) {
    return new FunctionalCommand(
        () -> armController.setSetpoint(targetDegrees),
        () -> {
          double PIDoutput = armController.calculate(getArmPositionDegrees());
          double kGoutput =
              armFeedForward.calculate(Units.degreesToRadians(getArmPositionDegrees()), 0.0);
          armMoveMotor.set(PIDoutput);
        },
        (interrupted) -> {
          armMoveMotor.set(0.0);
        },
        () -> false,
        this);
    // armMovePID.setSetpoint(targetDegrees, SparkBase.ControlType.kMAXMotionPositionControl,
    // ClosedLoopSlot.kSlot0, ffVolts));
  }

  public Command manualArmMove(double voltage) {
    return this.runEnd(
        () -> armMoveMotor.setVoltage(voltage),
        () -> {
          armMoveMotor.setVoltage(0);
        });
  }

  public Command intakeArmStop() {
    return this.runOnce(() -> armMoveMotor.set(0));
  }

  public Command intakeSpin(double voltage) {
    return this.runEnd(
        () -> spinIntakeMotor.setVoltage(voltage), () -> spinIntakeMotor.setVoltage(0));
  }

  public FunctionalCommand runIntake(double voltage) {
    return new FunctionalCommand(
        () -> {
          spinIntakeMotor.set(voltage);
        },
        () -> {},
        (interrupted) -> {
          spinIntakeMotor.set(0);
        },
        () -> false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Pivot/Current Position ", getArmPositionDegrees());
    SmartDashboard.putNumber("Intake/Pivot/Raw Position", relativeMoveEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Motor Output", armMoveMotor.getAppliedOutput());
    updateValues();
  }

  public void updateValues() {
    double currentkP = tunablekP;
    double currentkD = tunablekD;
    double currentkI = tunablekI;
    double currentkG = tunablekG;
    double currentMaxAccel = tunableMaxAccel;
    double currentMaxVelocity = tunableMaxVelocity;
    double currentStartPosition = startPosition;

    tunablekP = SmartDashboard.getNumber("Intake/Pivot/kP", tunablekP);
    tunablekI = SmartDashboard.getNumber("Intake/Pivot/kI", tunablekI);
    tunablekD = SmartDashboard.getNumber("Intake/Pivot/kD", tunablekD);
    tunablekG = SmartDashboard.getNumber("Intake/Pivot/kG", tunablekG);
    tunableMaxAccel = SmartDashboard.getNumber("Intake/Pivot/Max Accel", MAX_Acceleration);
    tunableMaxVelocity = SmartDashboard.getNumber("Intake/Pivot/Max Velocity", MAX_Velocity);

    if (currentkP != tunablekP
        || currentkD != tunablekD
        || currentkI != tunablekI
        || currentkG != tunablekG
        || currentMaxAccel != tunableMaxAccel
        || currentMaxVelocity != tunableMaxVelocity
        || currentStartPosition != startPosition) {
      System.out.println("UPDATING VALUES");
      armController.setPID(currentkP, 0.0, currentkD);
      armFeedForward.setKg(currentkG);

      // moveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
      // moveConfig.smartCurrentLimit(60); // No clue on this guy
      // moveConfig.closedLoop.maxMotion
      //         .cruiseVelocity(MAX_Velocity)
      //         .maxAcceleration(MAX_Acceleration)
      //         .allowedProfileError(MAX_LoopError);
      // moveConfig.closedLoop
      //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      //         .pid(tunablekP, tunablekI, tunablekD)
      //         .outputRange(kMinOutput, kMaxOutput); //Max Fraction of Output

      // armMoveMotor.configure(moveConfig, ResetMode.kResetSafeParameters,
      // PersistMode.kPersistParameters);

    }
  }
}
