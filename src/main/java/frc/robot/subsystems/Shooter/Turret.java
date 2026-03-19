package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  // Motors

  private final SparkFlex flyWheelMotor = new SparkFlex(ShooterConstants.flyWheelCanId, SparkFlex.MotorType.kBrushless);
  private final SparkFlex rollerMotor = new SparkFlex(ShooterConstants.rollerCanId, SparkFlex.MotorType.kBrushless);
 
  // Encoders

  private final RelativeEncoder flyWheelEncoder = flyWheelMotor.getEncoder();
  private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  // Controllers

  private final SparkClosedLoopController flyWheelController =
      flyWheelMotor.getClosedLoopController();
 
  private final SparkClosedLoopController rollerController =
      rollerMotor.getClosedLoopController();

  // PID

  private double flyWheeltP = 0.0;
  private double flyWheeltD = 0.0;
  private double flyWheeltV = .001802;
  private double flyWheeltS = .15;

  private double rollertP = 0.0;
  private double rollertD = 0.0;
  private double rollertV = 0.001822;
  private double rollertS = 0.31;

  //Tuneable Setpoints
  final LoggedNetworkNumber flywheelSetpoint =
    new LoggedNetworkNumber("Shooter/Flywheel/SetpointRPM", 0.0);

 final LoggedNetworkNumber rollerSetpoint =
    new LoggedNetworkNumber("Shooter/Roller/SetpointRPM", 0.0);


  // Feedforward

  private final SimpleMotorFeedforward flywheelFF =
      new SimpleMotorFeedforward(flyWheeltV, flyWheeltS);

  private final SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(rollertV, rollertS);

  public Turret() {

    // Flywheel config

    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    flywheelConfig.inverted(true);

    flywheelConfig.idleMode(IdleMode.kCoast);
    flywheelConfig.smartCurrentLimit(50);
    flywheelConfig.inverted(true);

    flywheelConfig.closedLoop.pid(flyWheeltP, 0, flyWheeltD);
    flyWheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Roller config

    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig.inverted(false);
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(40);

    rollerConfig.closedLoop.pid(rollertP, 0, rollertD);

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

 
   

    instantiateTunables();
  }

  private void instantiateTunables() {
    SmartDashboard.putNumber("Shooter/Flywheel/kP", flyWheeltP);
    SmartDashboard.putNumber("Shooter/Flywheel/kD", flyWheeltD);
    SmartDashboard.putNumber("Shooter/Flywheel/kV", flyWheeltV);
    SmartDashboard.putNumber("Shooter/Flywheel/kS", flyWheeltS);

    SmartDashboard.putNumber("Shooter/Roller/kP", rollertP);
    SmartDashboard.putNumber("Shooter/Roller/kD", rollertD);
    SmartDashboard.putNumber("Shooter/Roller/kV", rollertV);
    SmartDashboard.putNumber("Shooter/Roller/kS", rollertS);
   
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

  public FunctionalCommand setRPMs() {
    return new FunctionalCommand(
        () -> {
                  double flywheelRPM = flywheelSetpoint.get();
                  double rollerRPM = rollerSetpoint.get();

               double flywheelVolts = flywheelFF.calculate(flywheelRPM / 60);
                double rollerVolts = rollerFF.calculate(rollerRPM / 60.0);

                    flyWheelController.setSetpoint(
              flywheelRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, flywheelVolts);
          rollerController.setSetpoint(
              rollerRPM, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0, rollerVolts);
        },
        () -> {},
        (interrupted) -> {
          flyWheelMotor.set(0.0);
          rollerMotor.set(0.0);
        },
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

  public Command rawSpinShooter() {
    return this.runEnd(
        () -> {
          flyWheelMotor.set(-.5);
          rollerMotor.set(.5);
        },
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
    SmartDashboard.putNumber("Shooter/Flywheel/RPM", flyWheelMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Flywheel/Output", flyWheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Roller/RPM", rollerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter/Roller/Output", rollerMotor.getAppliedOutput());
  }

  public void updateValues() {
    double currentFlyWheelkP = flyWheeltP;
    double currentFlyWheelkD = flyWheeltD;
    double currentFlyWheelkV = flyWheeltV;
    double currentFlyWheelkS = flyWheeltS;

    double currentRollerkP = rollertP;
    double currentRollerkD = rollertD;
    double currentRollerkV = rollertV;
    double currentRollerkS = rollertS;


    flyWheeltP = SmartDashboard.getNumber("Shooter/Flywheel/kP", flyWheeltP);
    flyWheeltD = SmartDashboard.getNumber("Shooter/Flywheel/kD", flyWheeltD);
    flyWheeltV = SmartDashboard.getNumber("Shooter/Flywheel/kV", flyWheeltV);
    flyWheeltS = SmartDashboard.getNumber("Shooter/Flywheel/kS", flyWheeltS);

    rollertP = SmartDashboard.getNumber("Shooter/Roller/kP", rollertP);
    rollertD = SmartDashboard.getNumber("Shooter/Roller/kD", rollertD);
    rollertV = SmartDashboard.getNumber("Shooter/Roller/kV", rollertV);
    rollertS = SmartDashboard.getNumber("Shooter/Roller/kS", rollertS);

  
    SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    SparkFlexConfig rollerConfig = new SparkFlexConfig();
   

    if (currentFlyWheelkP != flyWheeltP
        || currentFlyWheelkD != flyWheeltD
        || currentFlyWheelkV != flyWheeltV
        || currentFlyWheelkS != flyWheeltS) {

      flywheelConfig.idleMode(IdleMode.kCoast);
      flywheelConfig.smartCurrentLimit(60);
      flywheelConfig.inverted(true);

      flywheelConfig.closedLoop.pid(flyWheeltP, 0, flyWheeltD);

      flyWheelMotor.configure(
          flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      flyWheelMotor.configure(
          flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    if (currentRollerkP != rollertP
        || currentRollerkD != rollertD
        || currentRollerkV != rollertV
        || currentRollerkS != rollertS) {
      rollerConfig.idleMode(IdleMode.kCoast);
      rollerConfig.smartCurrentLimit(45);

      rollerConfig.closedLoop.pid(rollertP, 0, rollertD);

      rollerMotor.configure(
          rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      rollerMotor.configure(
          rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

   
    }
  }
