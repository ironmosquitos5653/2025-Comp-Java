package frc.robot.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class GenerateRed {

  public static int OFFSET = 35;

  public static void main(String[] args) throws IOException {
    // System.out.println(Double.parseDouble("5"));
    generateRed("src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java");
  }

  private static void generateRed(String path) throws IOException {
    try (BufferedReader br = new BufferedReader(new FileReader(path))) {

      ArrayList<String> reefPoints = new ArrayList<>();
      boolean inReef = false;
      int brackets = 0;

      for (String line; (line = br.readLine()) != null; ) {
        if (line.contains("private void initBlueReefPoints")) {
          inReef = true;
        }
        if (inReef) {
          reefPoints.add(line);
          brackets += line.replaceAll("[^{]", "").length();
          brackets -= line.replaceAll("[^}]", "").length();
          System.out.println(brackets);
        }
        if (brackets == 0) {
          inReef = false;
        }
      }

      for (String line : reefPoints) {
        line = line.replaceAll("blue", "red").replaceAll("Blue", "Red");
        if (line.contains("Translation2d")) {
          // new Translation2d(6.971, 4)
          int paren = line.indexOf("Translation2d(") + 14;

          String start = line.substring(0, paren);
          String middle = line.substring(paren);
          double x = Double.parseDouble(middle.substring(0, middle.indexOf(","))) + OFFSET;
          String end = middle.substring(middle.indexOf(",") - 1);

          line = start + x + end;
        }
        System.out.println(line);
      }
    }
  }
}
