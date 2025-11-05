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

import cowlib.Util;

@Logged
public class RobotContainer {
  // Creates our controller
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system
  @NotLogged
  private final CommandXboxController driveStick = new CommandXboxController(0);

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

    // Scales the max drive speed when the elevator is enabled.
    var scaling = drivebase.getDriverMaxVelocity() * this.getElevatorSpeedRatio();
    xy[0] *= scaling;
    xy[1] *= scaling;

    return xy;
  }

  // The function used to scale the drive speed when the elevator is enabled.
  private double getElevatorSpeedRatio() {
    if (elevator.encoderPosition() > 70) {
      return 0.5;
    } else {
      return 1;
    }
  }

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

    Trigger leftTriggerLow = driveStick.leftTrigger(0.1);
    Trigger leftTriggerHigh = driveStick.leftTrigger(0.9);

    Trigger climberMode = new Trigger(() -> this.climberMode);

    leftTriggerLow
        .and(leftTriggerHigh.negate())
        .and(climberMode.negate()).whileTrue(Commands.parallel(
            elevator.collapseCommand(),
            coralWrist.highBranchesCommand(),
            coralIntake.intakeCommand()));

    leftTriggerHigh
        .and(climberMode.negate())
        .whileTrue(Commands.parallel(
            elevator.extendCommand(2),
            coralWrist.highBranchesCommand(),
            coralIntake.intakeCommand()));

    driveStick.rightTrigger()
        .and(climberMode.negate())
        .onTrue(fixCoral);

    /* Align Command Button Logic */
    Trigger semiAutoCancel = new Trigger(this::anyJoystickInput);

    var fullRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kBothRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kBothRumble, 0));

    var leftRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kLeftRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kLeftRumble, 0));

    var rightRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kRightRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kRightRumble, 0));
    var rightAlignCommand = Commands.parallel(
        rightRumbleCommand.withTimeout(0.5),
        new ReefAlignCommand(drivebase, ReefSide.RIGHT))
        .until(semiAutoCancel);
    // This Smart Dashboard value is used by the CANdleSystem.java subsystem
    // .andThen(() -> SmartDashboard.putBoolean("Aligned", false));
    driveStick.rightStick().toggleOnTrue(rightAlignCommand);
    driveStick.povRight().toggleOnTrue(rightAlignCommand);
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
