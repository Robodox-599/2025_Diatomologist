package frc.robot.util;

import static frc.robot.FieldConstants.*;

import com.google.gson.*;
import com.google.gson.stream.JsonWriter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.FieldConstants.Reef;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.HashMap;
import java.util.Map;

// IN TERMINAL: "./gradlew writePosesToChoreo"

public final class ChoreoVariableWriter {
  public static void writeToChoreo() {
    String filePath = "src/main/deploy/choreo/jaiden's autos 2025.chor"; // Path to your JSON file

    Map<String, Pose2d> namedPosesMap = new HashMap<>();

    String[] blueLeftNames = {
      "REEF_BLUE_A", "REEF_BLUE_C", "REEF_BLUE_E", "REEF_BLUE_G", "REEF_BLUE_I", "REEF_BLUE_K"
    };
    for (int i = 0; i < REEF_BLUE_LEFT.length; i++) {
      namedPosesMap.put(
          blueLeftNames[i],
          (REEF_BLUE_LEFT[i])
              .transformBy(
                  new Transform2d(
                      AutoAlignPoseGenerator.L4_REEF_FACE_OFFSET_AUTO, 0.0, new Rotation2d(0))));
    }
    String[] blueRightNames = {
      "REEF_BLUE_B", "REEF_BLUE_D", "REEF_BLUE_F", "REEF_BLUE_H", "REEF_BLUE_J", "REEF_BLUE_L"
    };
    for (int i = 0; i < REEF_BLUE_RIGHT.length; i++) {
      namedPosesMap.put(
          blueRightNames[i],
          REEF_BLUE_RIGHT[i].transformBy(
              new Transform2d(
                  AutoAlignPoseGenerator.L4_REEF_FACE_OFFSET_AUTO, 0.0, new Rotation2d(0))));
    }
    String[] blueMiddleNames = {
      "REEF_BLUE_AB", "REEF_BLUE_CD", "REEF_BLUE_EF", "REEF_BLUE_GH", "REEF_BLUE_IJ", "REEF_BLUE_KL"
    };
    for (int i = 0; i < REEF_BLUE_MIDDLE.length; i++) {
      namedPosesMap.put(blueMiddleNames[i], REEF_BLUE_MIDDLE[i]);
    }

    String[] redLeftNames = {
      "REEF_RED_A", "REEF_RED_C", "REEF_RED_E", "REEF_RED_G", "REEF_RED_I", "REEF_RED_K"
    };
    for (int i = 0; i < REEF_RED_LEFT.length; i++) {
      namedPosesMap.put(redLeftNames[i], REEF_RED_LEFT[i]);
    }

    String[] redRightNames = {
      "REEF_RED_B", "REEF_RED_D", "REEF_RED_F", "REEF_RED_H", "REEF_RED_J", "REEF_RED_L"
    };
    for (int i = 0; i < REEF_RED_RIGHT.length; i++) {
      namedPosesMap.put(redRightNames[i], REEF_RED_RIGHT[i]);
    }

    String[] redMiddleNames = {
      "REEF_RED_AB", "REEF_RED_CD", "REEF_RED_EF", "REEF_RED_GH", "REEF_RED_IJ", "REEF_RED_KL"
    };
    for (int i = 0; i < REEF_RED_MIDDLE.length; i++) {
      namedPosesMap.put(redMiddleNames[i], REEF_RED_MIDDLE[i]);
    }

    namedPosesMap.put("REEF_CENTER", new Pose2d(Reef.center, new Rotation2d(0)));

    try (FileReader reader = new FileReader(filePath)) {
      System.out.println("[ChoreoVariableWriter]: " + filePath + " read successfully.");
      // Parse the JSON content into a JsonObject
      JsonObject jsonObject = JsonParser.parseReader(reader).getAsJsonObject();
      namedPosesMap.forEach(
          (name, pose) -> {
            jsonObject
                .get("variables")
                .getAsJsonObject()
                .get("poses")
                .getAsJsonObject()
                .add(name, generateJsonPoseElement(pose));
          });

      reader.close();

      Gson gson = new GsonBuilder().setPrettyPrinting().create();

      try (FileWriter writer = new FileWriter(filePath)) {
        JsonWriter jsonWriter = new JsonWriter(writer);
        gson.toJson(jsonObject, jsonWriter);
        System.out.println("[ChoreoVariableWriter]: " + filePath + " updated successfully.");
      } catch (Exception e) {
        System.err.println("Error: Failed writing " + filePath);
      }
    } catch (Exception e) {
      System.err.println("Error: Failed reading or parsing " + filePath);
    }
  }

  private static JsonElement generateJsonPoseElement(Pose2d pose) {
    return JsonParser.parseString(
        "{"
            + generateExpression("x", pose.getX(), "m")
            + ","
            + generateExpression("y", pose.getY(), "m")
            + ","
            + generateExpression("heading", pose.getRotation().getRadians(), "rad")
            + "}");
  }

  private static String generateExpression(String name, Object value, String unitSI) {
    return "\"" + name + "\":{\"exp\":\"" + value + " " + unitSI + "\", \"val\":" + value + "}";
  }
}
