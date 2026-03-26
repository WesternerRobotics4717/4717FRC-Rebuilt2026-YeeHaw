package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShotMap {
  private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
  // private final Supplier<Double>

  public ShotMap() {
    // Distance is in Meters, Angle is in degrees, RPM is in RPM, duh,
    // 38.5 inches, 2500shooter, 2250roller, hood ~6
    // 90 + 23 + 13.5, 4000shooter, hood ~11.5
    // First number is distance, second number is either angle or rpm
    // Need to have values at least every half meter.
    angleMap.put(5.26, 22.0);
    angleMap.put(3.2, 14.0);
    angleMap.put(Units.inchesToMeters(38.5), 6.0);
    angleMap.put(Units.inchesToMeters(126.5), 11.5);

    rpmMap.put(5.26, 5000.0);
    rpmMap.put(3.2, 2850.0);
    rpmMap.put(Units.inchesToMeters(38.5), 2500.0);
    rpmMap.put(Units.inchesToMeters(126.5), 4000.0);
  }

  public double getGoodHoodAngle(double distanceMeters) {
    return angleMap.get(distanceMeters);
  }

  public double getGoodRPM(double distanceMeters) {
    return rpmMap.get(distanceMeters);
  }
}
