// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cowlib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
  public static final class DriveConstants {
    // Radius (maybe) of the area that the drivestick input has no effect
    public static final double deadband = 0.09;
    public static final double autoCancelThreshold = 0.35;

    public static final int currentLimit = 40;

    // Lower number for higher center of mass
    public static final double slewRate = 20;

    // Where to find drive reduction (for swerve):
    // (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
    // L1: 1.0 / 8.14 ; L2: 1.0 / 6.75 ; L3: 1.0 / 6.12
    // ALSO CHANGE IN COWLIB.SWERVEMODULE!!!!!!!!!!!!!!!!!!!!!! <----------
    public static final double driveGearing = 6.12;
    public static final double driveReduction = 1.0 / driveGearing;

    // Get from website, divide by 60 since the number that the
    // website gives is in rotations per minute and we want
    // rotations per second.
    public static final double neoFreeSpeed = 5820.0 / 60.0;

    // Measured by hand, write in meters
    public static final double wheelDiameter = 0.1016;

    // Max velocity = Fastest the motor can spin (free speed) times the gear ratio
    // times the circumference of the wheel (diameter (twice the radius) times pi).
    public static final double maxVelocity = neoFreeSpeed * driveReduction * wheelDiameter * Math.PI;

    // Max velocity applied tangentially (in a direction that causes the robot to
    // spin) divided by the radius of the robot (center to wheel) is equal to the
    // max angular velocity. Rearrangement of basic physics formula.
    public static final double maxAngularVelocity = maxVelocity / (ModuleLocations.robotRaduius);

    // This is the nominal battery voltage.
    public static final double maxVoltage = 12;

    // The PID loop that runs with our swerve. Don't touch unless necessary.
    public static final class SwervePID {
      public static final double p = 0.12;
      public static final double i = 0;
      public static final double d = 0.0015;
    }

    // Creates SwerveModules for use in Drivebase. Sets IDs for the motors and
    // encoder of each module, as well as direction of drive.
    public static final class SwerveModules {
      public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(1, 11, 21, true);
      public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(2, 12, 22, true);
      public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(3, 13, 23, true);
      public static final SwerveModuleConfig backRight = new SwerveModuleConfig(4, 14, 24, true);
    }

    public static final class ModuleLocations {
      // Measurement from center of one wheel to center of the other divided by 2. Ex:
      // this robot was 24 inches in length between center of wheels and 24.5 inches
      // in width.
      public static final double moduleLocationLength = Units.inchesToMeters(12.00);
      public static final double moduleLocationWidth = Units.inchesToMeters(12.25);

      // The robot's radius is the hypotenuse of the triangle with legs made by the
      // length and width defined above. Math is done automatically when the length
      // and width are changed.
      // Formula: square root((Length)^2 + (Width)^2) = Radius
      public static final double robotRaduius = Math
          .sqrt(Math.pow(moduleLocationLength, 2) + Math.pow(moduleLocationWidth, 2));

      // Locations of the wheels relative to the center of the robot.
      public static final Translation2d frontLeft = new Translation2d(moduleLocationLength, moduleLocationLength);
      public static final Translation2d frontRight = new Translation2d(moduleLocationLength, -moduleLocationLength);
      public static final Translation2d backLeft = new Translation2d(-moduleLocationLength, moduleLocationLength);
      public static final Translation2d backRight = new Translation2d(-moduleLocationLength, -moduleLocationLength);
    }

    public static final class PathPlannerConstants {
      public static final class TranslationPID {
        public static final double p = 1.55;
        public static final double i = 0.1;
        public static final double d = 1;
      }

      public static final class RotationPID {
        public static final double p = 1.5;
        public static final double i = 0;
        public static final double d = 0;
      }
    }

    public static final class ArmConstants {

      public static final int ArmMotorID = 6;
    }

    public static final class IntakeConstants {

      public static final int UpperIntakeMotorID = 9;
      public static final int LowerIntakeMotorID = 10;

      public static final int leftFoldingMotorID = 7;
      public static final int rightFoldingMotorID = 8;
    }
  }
}
