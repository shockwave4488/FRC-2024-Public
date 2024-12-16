package frc4488.lib.autonomous;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc4488.lib.math.EpsilonUtil;
import frc4488.robot.constants.Constants2024;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class PathPlannerUtil {
  public static class FlipPathsTask {
    private static final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    @SuppressFBWarnings(
        value = "NP_NULL_ON_SOME_PATH_FROM_RETURN_VALUE",
        justification = "pathplanner/paths and pathplanner/autos are folders")
    public static void main(String[] args) throws Exception {
      File pathplannerFolder = new File("src/main/pathplanner");
      File deployPathplannerFolder = new File("src/main/deploy/pathplanner");

      File deployPathsFolder = new File(deployPathplannerFolder, "paths");
      Files.createDirectories(deployPathsFolder.toPath());
      for (File path : new File(pathplannerFolder, "paths").listFiles()) {
        try {
          flipPath(
              path,
              new File(deployPathsFolder, "Blue-" + path.getName()),
              new File(deployPathsFolder, "Red-" + path.getName()));
        } catch (Exception e) {
          throw new Exception("Failed to flip path '" + path.getName() + "'", e);
        }
      }

      File deployAutosFolder = new File(deployPathplannerFolder, "autos");
      Files.createDirectories(deployAutosFolder.toPath());
      for (File auto : new File(pathplannerFolder, "autos").listFiles()) {
        try {
          flipAuto(
              auto,
              new File(deployAutosFolder, "Blue-" + auto.getName()),
              new File(deployAutosFolder, "Red-" + auto.getName()));
        } catch (Exception e) {
          throw new Exception("Failed to flip auto '" + auto.getName() + "'", e);
        }
      }
    }

    private static void flipPath(File source, File normalTarget, File flippedTarget)
        throws Exception {
      JsonObject normalJson = readFile(source);
      JsonObject flippedJson = normalJson.deepCopy();

      for (JsonElement waypointElement : flippedJson.get("waypoints").getAsJsonArray()) {
        JsonObject waypoint = waypointElement.getAsJsonObject();
        flipPos(waypoint.get("anchor"));

        JsonElement prevControl = waypoint.get("prevControl");
        if (!prevControl.isJsonNull()) {
          flipPos(prevControl);
        }

        JsonElement nextControl = waypoint.get("nextControl");
        if (!nextControl.isJsonNull()) {
          flipPos(nextControl);
        }
      }

      for (JsonElement rotationElement : flippedJson.get("rotationTargets").getAsJsonArray()) {
        JsonObject rotation = rotationElement.getAsJsonObject();
        rotation.addProperty(
            "rotationDegrees", flipRotation(rotation.get("rotationDegrees").getAsDouble()));
      }

      JsonArray normalEventMarkers = normalJson.get("eventMarkers").getAsJsonArray();
      JsonArray flippedEventMarkers = flippedJson.get("eventMarkers").getAsJsonArray();
      for (int i = 0; i < normalEventMarkers.size(); i++) {
        flipCommand(
            normalEventMarkers.get(i).getAsJsonObject().get("command").getAsJsonObject(),
            flippedEventMarkers.get(i).getAsJsonObject().get("command").getAsJsonObject());
      }

      JsonObject goalEndState = flippedJson.get("goalEndState").getAsJsonObject();
      goalEndState.addProperty(
          "rotation", flipRotation(goalEndState.get("rotation").getAsDouble()));

      JsonElement previewStartingStateElement = flippedJson.get("previewStartingState");
      if (!previewStartingStateElement.isJsonNull()) {
        JsonObject previewStartingState = previewStartingStateElement.getAsJsonObject();
        previewStartingState.addProperty(
            "rotation", flipRotation(previewStartingState.get("rotation").getAsDouble()));
      }

      Files.writeString(normalTarget.toPath(), gson.toJson(normalJson));
      Files.writeString(flippedTarget.toPath(), gson.toJson(flippedJson));
    }

    private static void flipAuto(File source, File normalTarget, File flippedTarget)
        throws Exception {
      JsonObject normalJson = readFile(source);
      JsonObject flippedJson = normalJson.deepCopy();

      JsonObject startingPose = flippedJson.get("startingPose").getAsJsonObject();
      flipPos(startingPose.get("position"));
      startingPose.addProperty(
          "rotation", flipRotation(startingPose.get("rotation").getAsDouble()));

      flipCommand(
          normalJson.get("command").getAsJsonObject(),
          flippedJson.get("command").getAsJsonObject());

      Files.writeString(normalTarget.toPath(), gson.toJson(normalJson));
      Files.writeString(flippedTarget.toPath(), gson.toJson(flippedJson));
    }

    private static void flipCommand(JsonObject normalJson, JsonObject flippedJson)
        throws Exception {
      String type = normalJson.get("type").getAsString();
      JsonObject normalData = normalJson.get("data").getAsJsonObject();
      JsonObject flippedData = flippedJson.get("data").getAsJsonObject();

      switch (type) {
        case "wait", "named" -> {}
        case "path" -> {
          String pathName = normalData.get("pathName").getAsString();
          normalData.addProperty("pathName", "Blue-" + pathName);
          flippedData.addProperty("pathName", "Red-" + pathName);
        }
        case "sequential", "parallel", "race", "deadline" -> {
          JsonArray normalCommands = normalData.get("commands").getAsJsonArray();
          JsonArray flippedCommands = flippedData.get("commands").getAsJsonArray();
          for (int i = 0; i < normalCommands.size(); i++) {
            flipCommand(
                normalCommands.get(i).getAsJsonObject(), flippedCommands.get(i).getAsJsonObject());
          }
        }
        default -> {
          throw new IOException("Invalid command type '" + type + "'");
        }
      }
    }

    private static JsonObject readFile(File file) throws Exception {
      try (FileReader reader = new FileReader(file, Charset.defaultCharset())) {
        JsonObject json = gson.fromJson(reader, JsonObject.class);
        if (json.has("version")
            && EpsilonUtil.epsilonEquals(json.get("version").getAsDouble(), 1.0)) {
          return json;
        }
        throw new IOException("Unsupported path/auto version! Expected 1.0");
      }
    }

    private static void flipPos(JsonElement pos) {
      JsonObject obj = pos.getAsJsonObject();
      obj.addProperty("y", Constants2024.FieldConstants.FIELD_WIDTH - obj.get("y").getAsDouble());
    }

    private static double flipRotation(double rotation) {
      return -rotation;
    }
  }

  public static Command buildAutoWithAlliance(String autoName) {
    return AutoBuilder.buildAuto(DriverStation.getAlliance().orElseThrow() + "-" + autoName);
  }

  public static Pose2d getAutoStartingPoseWithAlliance(String autoName) {
    return getAutoStartingPose(DriverStation.getAlliance().orElseThrow() + "-" + autoName);
  }

  // Mostly from com.pathplanner.lib.auto.AutoBuilder.buildAuto
  @SuppressFBWarnings(
      value = "DM_DEFAULT_ENCODING",
      justification = "Consistent with PathPlanner lib")
  private static Pose2d getAutoStartingPose(String autoName) {
    try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
      return AutoBuilder.getStartingPoseFromJson((JSONObject) json.get("startingPose"));
    } catch (AutoBuilderException e) {
      throw e;
    } catch (Exception e) {
      throw new RuntimeException(e.getMessage());
    }
  }
}
