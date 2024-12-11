package frc4488.lib.drive;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;

@SuppressFBWarnings("PA_PUBLIC_PRIMITIVE_ATTRIBUTE")
public class SwerveParameters {
  public int driveMotorChannel;
  public int turningMotorChannel;
  public int turningEncoderChannel;

  /** Angle encoder offset value. Should be in ticks for neos and radians for falcons */
  public double absoluteEncoderOffset;

  /**
   * Resolution of the absolute angle encoder used to determine the {@link #absoluteEncoderOffset}
   */
  public int absoluteEncoderResolution;

  /** Resolution of the potentiometer/encoder used for turning PID control */
  public int relativeTurningEncoderResolution;

  /** Resolution of the (likely integrated) encoder used for velocity control of the drive motor */
  public int driveEncoderResolution;

  public double wheelDiameter;

  /**
   * Gear ratio of the motor to wheel (how many times the motor spins for the wheel to spin once)
   */
  public double driveGearRatio;

  public double turnGearRatio;
  public ModulePosition modulePosition;
  public double moduleX;
  public double moduleY;

  public enum ModulePosition {
    FRONT_LEFT("FrontLeft", "Front Left"),
    FRONT_RIGHT("FrontRight", "Front Right"),
    BACK_LEFT("BackLeft", "Back Left"),
    BACK_RIGHT("BackRight", "Back Right");

    private final String compressedName;
    private final String expandedName;

    private ModulePosition(String compressedName, String expandedName) {
      this.compressedName = compressedName;
      this.expandedName = expandedName;
    }

    public String getCompressedName() {
      return compressedName;
    }

    public String getExpandedName() {
      return expandedName;
    }
  }

  public static SwerveParameters[] getAllFromJson(JsonObject json) {
    return json.entrySet().stream()
        .map(entry -> getFromJson(ModulePosition.valueOf(entry.getKey()), entry.getValue()))
        .toArray(SwerveParameters[]::new);
  }

  public static SwerveParameters getFromJson(ModulePosition pos, JsonElement json) {
    SwerveParameters parameters = new Gson().fromJson(json, SwerveParameters.class);
    parameters.modulePosition = pos;
    return parameters;
  }
}
