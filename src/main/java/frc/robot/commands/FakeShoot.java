// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;

public class FakeShoot extends Command {
  /** Creates a new PsuedoShoot. */
  public FakeShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.limelightSubsystem, RobotContainer.ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.limelightSubsystem.targetAreaReached()) {
      RobotContainer.ledSubsystem.ledColour(LEDConstants.BLUE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.ledSubsystem.ledColour(LEDConstants.WHITE);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!RobotContainer.limelightSubsystem.targetAreaReached()) {
      return true;
    }
    return false;
  }
}
