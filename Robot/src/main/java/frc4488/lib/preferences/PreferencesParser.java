package frc4488.lib.preferences;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc4488.lib.logging.LogManager;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.Function;

public class PreferencesParser {
  public static final Path CONFIG_DIR_TXT =
      Filesystem.getOperatingDirectory()
          .toPath()
          .resolve((RobotBase.isReal() ? "" : "simulation" + File.separator) + "configDir.txt");

  private final JsonObject json;
  private final LogManager logger;

  public PreferencesParser(LogManager logger) {
    this.logger = logger;

    String prefsDir = "";
    try {
      prefsDir = Files.readString(CONFIG_DIR_TXT);
    } catch (IOException e) {
      System.out.print(
          "Could not find preferences directory from "
              + CONFIG_DIR_TXT.toAbsolutePath().toString()
              + ". ");
      if (RobotBase.isReal()) {
        System.out.println(
            "Did you forget to send a 'configDir.txt' file containing the prefs location to the RoboRIO's home directory?");
      } else {
        System.out.println(
            "Did you forget to create 'simulation/configDir.txt' to specify the prefs location?");
      }
      json = new JsonObject();
      return;
    }

    String preferencesPathString =
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve(prefsDir + File.separator + "Preferences.json")
            .toString();

    try (FileReader fileReader = new FileReader(preferencesPathString, Charset.forName("UTF-8"))) {
      json = new Gson().fromJson(fileReader, JsonObject.class);
    } catch (FileNotFoundException e) {
      throw new RuntimeException("Could not find preferences file at " + preferencesPathString);
    } catch (IOException e) {
      throw new RuntimeException("IOException in PreferencesParser", e);
    }
  }

  public String getString(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonPrimitive() && element.getAsJsonPrimitive().isString()) {
        return element.getAsString();
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public boolean getBoolean(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonPrimitive() && element.getAsJsonPrimitive().isBoolean()) {
        return element.getAsBoolean();
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public int getInt(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonPrimitive() && element.getAsJsonPrimitive().isNumber()) {
        return element.getAsInt();
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public int[] getIntArray(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonArray()) {
        try {
          return element.getAsJsonArray().asList().stream()
              .mapToInt(JsonElement::getAsInt)
              .toArray();
        } catch (AssertionError e) {
          // Happens when JsonElement#getAsInt is called on a non-number
        }
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public double getDouble(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonPrimitive() && element.getAsJsonPrimitive().isNumber()) {
        return element.getAsDouble();
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public JsonObject getJsonObject(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonObject()) {
        return element.getAsJsonObject();
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public JsonArray getJsonArray(String key) throws PreferenceDoesNotExistException {
    if (json.has(key)) {
      JsonElement element = json.get(key);
      if (element.isJsonArray()) {
        return element.getAsJsonArray();
      }
    }

    throw new PreferenceDoesNotExistException(key, logger);
  }

  public <T> T tryGetValue(Function<String, T> valueGetter, String key, T defaultValue) {
    try {
      return valueGetter.apply(key);
    } catch (PreferenceDoesNotExistException e) {
      return defaultValue;
    }
  }
}
