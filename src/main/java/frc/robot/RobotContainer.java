// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivebase;
import cowlib.Util;

@Logged
public class RobotContainer {
  // Creates our controller
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system
  @NotLogged
  private final CommandXboxController driveStick = new CommandXboxController(0);
  private final Arm Arm = new Arm();

  // Creates our subsystems
  private final Drivebase drivebase = new Drivebase();

  // This, plus some lines below, creates a drop down menu on the dashboard for
  // selecting our auto.
  private SendableChooser<Command> autoChooser;

  // Tells the robot to drive by default.
  public RobotContainer() {
    drivebase.setDefaultCommand(
        new Drive(
            drivebase,
            this::getScaledXY,
            () -> scaleRotationAxis(driveStick.getRightX())));

    this.configureBindings();
  }

  // Used to create an area around the center of the joystick where the input is
  // 0, so as to avoid stick drift.
  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  // Changes our input -> output from linear to exponential, allowing finer
  // control close to the center without limiting our max output (since 1 is the
  // highest input and 1^2 (the ouput) = 1, so no change to the edges of the
  // input/output)
  private double[] getScaledXY() {
    // Array for storing the x/y inputs from the controller
    double[] xy = new double[2];

    // Assigning inputs to array locations. X and Y are switched because the
    // controller is funky.
    xy[0] = deadband(-driveStick.getLeftY(), DriveConstants.deadband);
    xy[1] = deadband(-driveStick.getLeftX(), DriveConstants.deadband);

    Util.square2DVector(xy);

    return xy;
  }

  // The function used to scale the drive speed when the elevator is enabled.

  private double scaleRotationAxis(double input) {
    return this.deadband(this.squared(input), DriveConstants.deadband) * drivebase.getMaxAngularVelocity() * -0.6;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  public void resetGyro() {
    drivebase.resetGyroCommand();
  }

  private void configureBindings() {
    /* Intake/Output buttons */
    // Intake

    driveStick.a().onTrue(Arm.highPosCommand());
    driveStick.b().onTrue(Arm.lowPosCommand());

    driveStick.x().whileTrue(Arm.inTake());
  }

  private boolean anyJoystickInput() {
    return deadband(driveStick.getLeftY(), DriveConstants.autoCancelThreshold) != 0
        || deadband(driveStick.getLeftX(), DriveConstants.autoCancelThreshold) != 0
        || deadband(driveStick.getRightX(), DriveConstants.autoCancelThreshold) != 0;
  }

  public Command getAutonomousCommand() {
    // This, plus some lines above, creates a drop down menu on the dashboard for
    // selecting our auto.
    return this.autoChooser.getSelected();
  }
}
