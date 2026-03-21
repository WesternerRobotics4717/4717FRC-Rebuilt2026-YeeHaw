package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotMap {
  private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

  public ShotMap() {
    // Distance is in Meters, Angle is in degrees, RPM is in RPM, duh
    // First number is distance, second number is either angle or rpm
    angleMap.put(5.26, 22.0);
    angleMap.put(3.2, 14.0);

    rpmMap.put(5.26, 5000.0);
    rpmMap.put(3.2, 2850.0);
  }

  public double getGoodHoodAngle(double distanceMeters) {
    return angleMap.get(distanceMeters);
  }

  public double getGoodRPM(double distanceMeters) {
    return rpmMap.get(distanceMeters);
  }
}
