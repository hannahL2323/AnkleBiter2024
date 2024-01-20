// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // public static CameraSubsystem cameraSubsystem = new CameraSubsystem();
  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  // driver xbox
  XboxController driverController = new XboxController(0);

  JoystickButton driverA = new JoystickButton(driverController, 1);
  JoystickButton driverB = new JoystickButton(driverController, 2);
  JoystickButton driverX = new JoystickButton(driverController, 3);
  JoystickButton driverY = new JoystickButton(driverController, 4);
  JoystickButton driverLeftBumper = new JoystickButton(driverController, 5);
  JoystickButton driverRightBumper = new JoystickButton(driverController, 6);
  
  // driver joystick
  Joystick driverJoystick = new Joystick(0);
  JoystickButton joystickTrigger = new JoystickButton(driverJoystick, 1);
  JoystickButton thumbButton = new JoystickButton(driverJoystick, 2);
  JoystickButton joystickA = new JoystickButton(driverJoystick, 3);
  JoystickButton joystickB = new JoystickButton(driverJoystick, 4);
  JoystickButton joystickX = new JoystickButton(driverJoystick, 5);
  JoystickButton joystickY = new JoystickButton(driverJoystick, 6);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
    //                                                       () -> MathUtil.applyDeadband(driverController.getLeftY(),
    //                                                                                    OperatorConstants.LEFT_Y_DEADBAND),
    //                                                       () -> MathUtil.applyDeadband(driverController.getLeftX(),
    //                                                                                    OperatorConstants.LEFT_X_DEADBAND),
    //                                                       () -> -driverController.getRightX(),
    //                                                       () -> -driverController.getRightY(),
    //                                                       false);

    // AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
    //                                                                      () ->
    //                                                                          MathUtil.applyDeadband(driverController.getLeftY(),
    //                                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                      () -> MathUtil.applyDeadband(driverController.getLeftX(),
    //                                                                                                   OperatorConstants.LEFT_X_DEADBAND),
    //                                                                      () -> driverController.getRawAxis(2), false);

    // TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
    //                                                 () -> MathUtil.applyDeadband(driverController.getLeftY(),
    //                                                                              OperatorConstants.LEFT_Y_DEADBAND),
    //                                                 () -> MathUtil.applyDeadband(driverController.getLeftX(),
    //                                                                              OperatorConstants.LEFT_X_DEADBAND),
    //                                                 () -> driverController.getRawAxis(2), () -> true, false, true);
    TeleopDrive xBoxTeleopDrive = new TeleopDrive(
      drivebase,
      () -> MathUtil.applyDeadband(-driverController.getLeftY() * 0.7, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverController.getLeftX() * 0.7, OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverController.getRightX() * 0.7, () -> true);
    
    TeleopDrive joystickTeleopDrive = new TeleopDrive(
      drivebase, 
      () -> MathUtil.applyDeadband(-driverJoystick.getY() * 0.7, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverJoystick.getX() * 0.7, OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-driverJoystick.getTwist() * 0.7, OperatorConstants.ROTATION_DEADBAND), () -> true);

    // drivebase.setDefaultCommand(xBoxTeleopDrive);
    drivebase.setDefaultCommand(joystickTeleopDrive);
    
  }

  
  

  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // driverLeftBumper.whileTrue(new TeleopDrive(
    //     drivebase,
    //     () -> cameraSubsystem.getDriveSpeed() * 0.5,
    //     () -> cameraSubsystem.getTurnSpeed() * 0.5,
    //     () -> 0, () -> true));

    driverA.onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverController, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    driverX.whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

}
