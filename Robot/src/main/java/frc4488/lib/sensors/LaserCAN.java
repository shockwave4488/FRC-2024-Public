package frc4488.lib.sensors;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;

public class LaserCAN {
  private final LaserCan lc;

  public LaserCAN(int port) {
    lc = new LaserCan(port);
  }

  public int getDistance() {
    Measurement distance_mm = lc.getMeasurement();
    if (distance_mm == null) {
      return Integer.MAX_VALUE;
    } else {
      return distance_mm.distance_mm;
    }
  }
}
