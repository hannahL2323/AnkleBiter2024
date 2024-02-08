// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class DriveToTag extends Command {

  private final SwerveSubsystem  swerve;
  // private final DoubleSupplier   vX;
  // private final DoubleSupplier   vY;
  // private final DoubleSupplier   omega;
  // private final BooleanSupplier  driveMode;

  // NetworkTable for the Limelight camera (contains list of NetworkTableEntries, which are bits of a continuous data stream)
  NetworkTable table;
  // NetworkTable data stream (aka an entry) for horizontal angle displacement, vertical angle displacement, and target area
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  // Variables for storing the Limelight NetworkTable stream updates
  double xDisplacement;
  double yDisplacement;
  double targetArea;

  double x;
  double y;
  double area;

  final double DESIRED_AREA;

  SwerveController controller;


  /** Creates a new DriveToTag. */
  public DriveToTag(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerve = swerve;
    // this.vX = vX;
    // this.vY = vY;
    // this.omega = omega;
    // this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    
    addRequirements(RobotContainer.limelightSubsystem, swerve);

    swerve = RobotContainer.swerveSubsystem;

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    DESIRED_AREA = 15;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // double xVelocity   = Math.pow(y, 3);
    double yVelocity   = Math.pow(y, 3);
    double angVelocity = Math.pow(x, 3);

    // Drive using raw values.
    swerve.drive(new Translation2d(yVelocity * 0.2, yVelocity * 0.2),
                 angVelocity * 0.02,
                 true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (x <= 5 || y <= 5 || (area - DESIRED_AREA) <= 1 ) {
    //   return true;
    // }
    return false;
  }
}
