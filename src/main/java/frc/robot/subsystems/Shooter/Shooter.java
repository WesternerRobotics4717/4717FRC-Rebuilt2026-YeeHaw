package frc.robot.subsystems.Shooter;

import java.util.List;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DeviceIDs.ShooterConstants;

public class Shooter extends SubsystemBase {

    // Motors

    private final SparkFlex shooterFlyWheel = new SparkFlex(ShooterConstants.flyWheelCanId, SparkFlex.MotorType.kBrushless);
    private final SparkFlex shooterRollers = new SparkFlex(ShooterConstants.shooterRollersCanId, SparkFlex.MotorType.kBrushless);
    private final SparkFlex shooterHoodController = new SparkFlex(ShooterConstants.hoodCanId, SparkFlex.MotorType.kBrushless);
    

    // Encoders

    private final RelativeEncoder flyWheelEncoder = shooterFlyWheel.getEncoder();
    private final RelativeEncoder hoodEncoder;
    private final RelativeEncoder rollerEncoder = shooterRollers.getEncoder();

    // Controllers

    private final SparkClosedLoopController flyWheelController = shooterFlyWheel.getClosedLoopController();
    private final SparkClosedLoopController hoodController = shooterHoodController.getClosedLoopController();
    private final SparkClosedLoopController rollerController = shooterRollers.getClosedLoopController();

    


    // PID

    private double flyWheeltP = 0.2;
    private double flyWheeltD = 0.001;
    private double flyWheeltV = .001802;
    private double flyWheeltS = .15;

    private double rollertP = 0.2;
    private double rollertD = 0.001;
    private double rollertV = 0.001822;
    private double rollertS = 0.31;

    private double hoodtP = 0.012;
    private double hoodtD = 0.0;

    // Feedforward

    private final SimpleMotorFeedforward flywheelFF =
            new SimpleMotorFeedforward(flyWheeltV, flyWheeltS);

    private final SimpleMotorFeedforward rollerFF = new SimpleMotorFeedforward(rollertV, rollertS);
   

    public Shooter() {

        // Flywheel config

        SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        flywheelConfig.inverted(true);

        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.smartCurrentLimit(50);
        flywheelConfig.inverted(true);

        flywheelConfig.closedLoop
               .pid(flyWheeltP, 0, flyWheeltD);
        shooterFlyWheel.configure(
                flywheelConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // Roller config

        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.inverted(false);
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(40);

        rollerConfig.closedLoop.pid(rollertP, 0, rollertD);

        shooterRollers.configure(
                rollerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // Hood config

        SparkFlexConfig hoodConfig = new SparkFlexConfig();

        hoodConfig.idleMode(IdleMode.kBrake);

        hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(hoodtP,0, hoodtD);

        hoodConfig.encoder.positionConversionFactor(0.125*360);


        shooterHoodController.configure(
                hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
                
        );

        hoodEncoder = shooterHoodController.getEncoder();
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

        SmartDashboard.putNumber("Shooter/Hood/kP", hoodtP);
        SmartDashboard.putNumber("Shooter/Hood/kD", hoodtD);


        
    }
    

     

    // Flywheel control

     public FunctionalCommand setFlywheelRPM(double rpm) {
        double ffVolts = flywheelFF.calculate(rpm / -60.0);


       return new FunctionalCommand(
        () -> {
            flyWheelController.setSetpoint(
                rpm,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts);
        }, 
        () -> {}, 
        (interrupted) -> {
            shooterFlyWheel.set(0.0);
        },
        () -> false);
    } 

    public FunctionalCommand setRPMs(double flywheelRPM, double acceleratorRPM) {
        double flywheelVolts = flywheelFF.calculate(flywheelRPM/60);
        double rollerVolts = rollerFF.calculate(acceleratorRPM/60.0);

        return new FunctionalCommand(
            () -> {
                flyWheelController.setSetpoint(
                    flywheelRPM,
                    SparkBase.ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    flywheelVolts);
                rollerController.setSetpoint(
                    acceleratorRPM,
                    SparkBase.ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    rollerVolts);
                
            }, 
            () -> {}, 
            (interrupted) -> {
                shooterFlyWheel.set(0.0);
                shooterRollers.set(0.0);},
            () -> false, this);
    }

    public double getFlywheelRPM() {
        return flyWheelEncoder.getVelocity();
    }


    // Roller control

        public Command setRollerRPM(double rpm) {
        double ffVolts = rollerFF.calculate(rpm / 60.0);


       return this.run(() -> rollerController.setSetpoint(
                rpm,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts
        )); 
    } 



    public double getRollerRPM() {
        return rollerEncoder.getVelocity();
    }

    // Hood control

    public FunctionalCommand setHoodAngle(double angle) {
        return new FunctionalCommand(
            () -> {
                hoodController.setSetpoint(
                angle,
                SparkBase.ControlType.kPosition);
                
            }, 
            () -> {}, 
            (interrupted) -> {
                shooterHoodController.set(0.0);
            },
            () -> false);
    }

    public double getHoodAngle() {
        return hoodEncoder.getPosition();
    } 

    public double getRollerVelocity() {
        return rollerEncoder.getVelocity();
    }


    public Command spinShooter(double wheelRPMs) {
        return this.runEnd(() -> {
            setFlywheelRPM(wheelRPMs);
            setRollerRPM(wheelRPMs);
        }, () -> {
                setFlywheelRPM(0);
                setRollerRPM(0);
        }
        );
    }

    public Command rawSpinShooter() {
        return this.runEnd(() -> {
            shooterFlyWheel.set(-.5);
            shooterRollers.set(.5);
        }, () -> {shooterFlyWheel.set(0);
                shooterRollers.set(0);
        }
        );
    }

    public Command rawMoveHood(double speed) {
        return this.runEnd(() -> shooterHoodController.set(speed)
        , () -> shooterHoodController.set(speed)
        );
    }


   /*  public record shotPositions(
        double flywheelRPM
        double distanceToHub
        
    ) {}

    List<shotPositions> ShotMap = List.of(


    ); */
    



    public void periodic() {
        updateValues();
        updateOdometry();
    }

    public void updateOdometry() {
        SmartDashboard.putNumber("Shooter/Flywheel/RPM", shooterFlyWheel.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Flywheel/Output", shooterFlyWheel.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Roller/RPM", shooterRollers.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Roller/Output", shooterRollers.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Hood/Position", getHoodAngle());
        SmartDashboard.putNumber("Shooter/Hood/Output", shooterHoodController.getAppliedOutput());
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

        double currentHoodkP = hoodtP;
        double currentHoodkD = hoodtD;

        
        
        flyWheeltP = SmartDashboard.getNumber("Shooter/Flywheel/kP", flyWheeltP);
        flyWheeltD = SmartDashboard.getNumber("Shooter/Flywheel/kD", flyWheeltD);
        flyWheeltV = SmartDashboard.getNumber("Shooter/Flywheel/kV", flyWheeltV);
        flyWheeltS = SmartDashboard.getNumber("Shooter/Flywheel/kS", flyWheeltS);



        rollertP = SmartDashboard.getNumber("Shooter/Roller/kP", rollertP);
        rollertD = SmartDashboard.getNumber("Shooter/Roller/kD", rollertD);
        rollertV = SmartDashboard.getNumber("Shooter/Roller/kV", rollertV);
        rollertS = SmartDashboard.getNumber("Shooter/Roller/kS", rollertS);


        hoodtP = SmartDashboard.getNumber("Shooter/Hood/kP", hoodtP);
        hoodtD = SmartDashboard.getNumber("Shooter/Hood/kD", hoodtD);



       SparkFlexConfig flywheelConfig = new SparkFlexConfig();
       SparkFlexConfig rollerConfig = new SparkFlexConfig(); 
       SparkFlexConfig hoodConfig = new SparkFlexConfig();

        if (currentFlyWheelkP != flyWheeltP || currentFlyWheelkD != flyWheeltD ||
              currentFlyWheelkV != flyWheeltV || 
            currentFlyWheelkS != flyWheeltS) {

                flywheelConfig.idleMode(IdleMode.kCoast);
                flywheelConfig.smartCurrentLimit(60);
                flywheelConfig.inverted(true);
        
                flywheelConfig.closedLoop
                       .pid(flyWheeltP, 0, flyWheeltD);
        
                shooterFlyWheel.configure(
                        flywheelConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters
                );
                shooterFlyWheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            }

        if (currentRollerkP != rollertP || currentRollerkD != rollertD ||
        currentRollerkV != rollertV || 
      currentRollerkS != rollertS) {
            rollerConfig.idleMode(IdleMode.kCoast);
            rollerConfig.smartCurrentLimit(45);
    
            rollerConfig.closedLoop.pid(rollertP, 0, rollertD);
    
            shooterRollers.configure(
                    rollerConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );
            shooterRollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        if(currentHoodkP != hoodtP || currentHoodkD != hoodtD) 
        {

            hoodConfig.idleMode(IdleMode.kBrake);
                    hoodConfig.encoder.positionConversionFactor(0.125*360);


            hoodConfig.closedLoop
                    .pid(hoodtP,0, hoodtD);
    
            shooterHoodController.configure(
                    hoodConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                    
            );
        }

}
}
