// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.AbsoluteFieldDrive;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.TeleopDrive;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  XboxController driverController = new XboxController(0);


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
    TeleopDrive teleopDrive = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(-driverController.getLeftY() * 0.7, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getLeftX() * 0.7, OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRightX() * 0.7, () -> true);
       
    System.out.println(driverController.getLeftY());
    System.out.println(driverController.getLeftX());
    System.out.println(driverController.getRightX());
    
    drivebase.setDefaultCommand(teleopDrive);

    
  }

  
  

  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverController, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverController, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverController, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
  }

}
