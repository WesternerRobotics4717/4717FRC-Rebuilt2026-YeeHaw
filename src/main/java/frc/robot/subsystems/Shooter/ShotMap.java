package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotMap {
  private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  // private final Supplier<Double>

  public ShotMap() {
    // Distance is in Meters, Angle is in degrees, RPM is in RPM, duh,
    // Furthest Point is 230 inches away
    // 84
    // First number is distance, second number is either angle or rpm
    // Need to have values at least every half meter.
    // 124

    angleMap.put(Units.inchesToMeters(55.0), 6.0);
    angleMap.put(Units.inchesToMeters(85.0), 8.0);
    angleMap.put(Units.inchesToMeters(112.0), 10.0);
    angleMap.put(Units.inchesToMeters(145.0), 15.0);
    angleMap.put(Units.inchesToMeters(216), 22.0);

    rpmMap.put(Units.inchesToMeters(55.0), 2750.0);
    rpmMap.put(Units.inchesToMeters(85.0), 3200.0);
    rpmMap.put(Units.inchesToMeters(112.0), 3400.0);
    rpmMap.put(Units.inchesToMeters(145.0), 3600.0);
    rpmMap.put(Units.inchesToMeters(216), 4500.0);
  }

  public double getGoodHoodAngle(double distanceMeters) {
    return angleMap.get(distanceMeters);
  }

  public double getAngle() {
    double distance = SmartDashboard.getNumber("AutoAim/TargetDistance", 0.0);
    return angleMap.get(distance);
  }

  public double getGoodRPM(double distanceMeters) {
    return rpmMap.get(distanceMeters);
  }

  public double getRPM() {
    double distance = SmartDashboard.getNumber("AutoAim/TargetDistance", 0.0);
    return rpmMap.get(distance);
  }
}
