// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Set;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static final class IntakeConstants {
    public static final int MOTOR_ID = 31;
    public static final int MOTOR_ID2 = 32;
    public static final int MOTOR_ID3 = 33;
    public static final int MAX_VOLTAGE = 12; // 12??
  }

  public static final class ShooterConstants {
    public static final int MOTOR_ID = 41; 
    public static final int HELPER_ID = 42;
    public static final int  MAX_VOLTAGE = 12;

    //PIDConstants -- tune later
    public static double kP = 2.5;
    public static double kI = 0.0;
    public static double kD = 0.1;
  }

  public static final class IntakePivotConstants {
    public static final int MOTOR_ID = 51;
    public static final int ENCODER_ID = 54;
    public static final int MAX_VOLTAGE = 12;


    //DEFINITELY CHANGE, THIS IS A GUESS!!
    public static final double INTAKE_POSITION = -.2;//-.27
    public static final double INTAKE_STOW = .35;//.377


    //tune pid and svag later
    public static final double kP = 2; // 7.5
    public static final double kI = 0.0;
    public static final double kD = 0.01; // 0.05

    public static final double kS = 0.11237;
    public static final double kV = 0.56387;
    public static final double kA = 0.041488;
    public static final double kG = 0.76416;
  }

  public static final class TurretConstants {
    public static final int MOTOR_ID = 21;
    public static final int MAX_VOLTAGE = 4;

    public static final double MAX_POSITION = 7;
    public static final double LIMELIGHT_ANGLE = 33; //WILL NEED CHANGE
    public static final double LIMELIGHT_HEIGHT_METERS = 19; //WILL NEED CHANGE
    public static final double TAG_HEIGHT_METERS = 44; //WILL NEED CHANGE
    public static final double DISTANCE_TAG_TO_TARGET = 23.75;
    public static final double DISTANCE_TAG_SIDE_TO_TARGET = 27.56923104d;
    public static final double DISTANCE_BETWEEN_TAGS = 14.0;
    public static final double alpha = Math.abs(Math.toDegrees(Math.acos(DISTANCE_BETWEEN_TAGS / DISTANCE_TAG_SIDE_TO_TARGET)));
    public static final int[][] cIds = new int[][]{{10, 9}};//change later
    public static final int[] allId = new int[] {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};//change later



    //tune pid and svag later
    public static final double kP = 2.7; // 2.7
    public static final double kI = 0.0;
    public static final double kD = 0.03; // 0.02

    public static final double kS = 0.11237;
    public static final double kV = 0.56387;
    public static final double kA = 0.041488;
    public static final double kG = 0.76416;


    //Most likely not needed later
    public static final double posO = 0d;
    public static final double pos1 = 5d;
    public static final double pos2 = 10d;
    public static final double pos3 = 20d;

  }

  public static final class HoodConstants {
    public static final int MOTOR_ID = 43;
    public static final int MAX_VOLTAGE = 9;

    public static final double BLUE_HUB_X = 4.6047;
    public static final double BLUE_HUB_Y = 4.0213;
    public static final double RED_HUB_X = 11.9083;
    public static final double RED_HUB_Y = 4.0213;
    

    public static double kP = 2.25; // 2
    public static double kI = 0.0;
    public static double kD = 0;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public static final class VisionConstants {
    // public static final double alignSpeed = -.5;
    // public static final double alignRange = 5;
    // public static final double kP = .04;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-static"; // this is the right LL
    // public static String camera1Name = "limelight-right"; // this is the left LL

    public static final double LIMELIGHT_HEIGHT_METERS = 0.65;
    public static final double LIMELIGHT_PITCH_DEG = 25.0;
    // Example target tags
    public static final Set<Integer> TARGET_TAG_IDS = Set.of(21, 22, 7);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = .75;

    // Standard deviation baselines, for 1-meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = .5; // More stable than full 3D solve .5
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available

    public static double maxAvgTagDistance = 3; // was 2.0// change this later

    // Boolean for Left/Right Reef
    public static boolean k_isRightReef = true;

    // Boolean for Committing to Shoot
    public static boolean k_positioned = true;

    // PID for Tag Relative Control for Scoring
    public static final double kP_aim = 0.15; // .1
    public static final double kI_aim = 0.0;
    public static final double kD_aim = 0.0;

    public static final double kP_range = 0.75; // .6
    public static final double kI_range = 0.0;
    public static final double kD_range = 0.0;

    public static final double kP_strafe = 0.75; // .6
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0;

    // AimNRange Reef Right
    public static final double k_aimReefRightTarget = 2.3; // 2.5
    public static final double k_rangeReefRightTarget = -0.45; // -.4
    public static final double k_strafeReefRightTarget = 0.19; // .18

    // AimNRange Reef Left
    public static final double k_aimReefLeftTarget = 4.4; // 4.6
    public static final double k_rangeReefLeftTarget = -.45; // -.45
    public static final double k_strafeReefLeftTarget = -0.32; // -.25

    // Prerequisites

    public static final double k_tzValidRange = -1.5;
    public static final double k_yawValidRange = 35;

    // Thresholds
    public static final double k_rangeThreshold = 0.02;
    public static final double k_strafeThreshold = 0.02;
    public static final double k_aimThreshold = 0.5;

    // For testing
    public static boolean k_positioning = false;
  }

 
}
