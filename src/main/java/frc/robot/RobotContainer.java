package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Swerve.SwerveDriveCommand;
import frc.robot.subsystems.Swerve;


public class RobotContainer {
  //Subsystems
  Swerve swerveDrivetrain = new Swerve(true);

  //Joysticks
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);

  //Commands
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);


  public RobotContainer() {
    swerveDrivetrain.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }


  private void configureButtonBindings() {}



  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
