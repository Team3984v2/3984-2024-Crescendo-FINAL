// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve.climber;
import frc.robot.Constants.Swerve.flywheel;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.aimAtTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmSetpointDegrees = 75.0;
  // Initialize control boards
  private final CommandXboxController driver = new CommandXboxController(1);
  private final CommandXboxController second = new CommandXboxController(0);
  // Initialize subsystems 
  private final PhotonCamera cam = new PhotonCamera("Global_Shutter_Camera");
  private final PhotonCamera cam2 = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private final Swerve s_Swerve = new Swerve(cam);
  private final Flywheel fwheel = new Flywheel();

  private final aimAtTarget aimCommand = new aimAtTarget(cam, s_Swerve, s_Swerve::getPose);
  private final SendableChooser<Command> autoChooser;
  private final Intake intake = new Intake();  
  private final Climber c_climber = new Climber();

    /* The container for the robot. subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Launch", intake.Out().withTimeout(0.2));
    NamedCommands.registerCommand("StartIntake", intake.In().withTimeout(0.00001));
    NamedCommands.registerCommand("IntakeDown", intake.moveTo(Constants.Swerve.intake.INTAKE, true));
    NamedCommands.registerCommand("StopIntake", intake.Stop().withTimeout(0.00001)); 
    NamedCommands.registerCommand("IntakeUp", intake.moveTo(Constants.Swerve.intake.IDLE, false));
    NamedCommands.registerCommand("SpinUp", fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX(),
        ()->false,// Robot centric swerve, doesnt work as of now
        () -> true));//driver.leftBumper().getAsBoolean())); // Slow mode toggle
    intake.setDefaultCommand(intake.Stop());
      //intake.moveTo(Constants.Swerve.intake.IDLE, false)
    fwheel.setDefaultCommand(fwheel.stop());
    // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //aim.whileTrue(aimCommand); // Orients the robot torwards april tag
    driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    //driver.a().and(driver.rightBumper()).whileTrue(s_Swerve.angleSysIdDynamic(SysIdRoutine.Direction.kForward));
    //driver.b().and(driver.rightBumper()).whileTrue(s_Swerve.angleSysIdDynamic(SysIdRoutine.Direction.kReverse));
    //driver.x().and(driver.rightBumper()).whileTrue(s_Swerve.angleSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //driver.y().and(driver.rightBumper()).whileTrue(s_Swerve.angleSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //driver.a().and(driver.leftBumper()).whileTrue(s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
    //driver.b().and(driver.leftBumper()).whileTrue(s_Swerve.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));
    //driver.x().and(driver.leftBumper()).whileTrue(s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //driver.y().and(driver.leftBumper()).whileTrue(s_Swerve.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    second.y().whileTrue(fwheel.ampShot());
    second.leftTrigger(0.3).whileTrue(fwheel.moveTo(flywheel.SPEAKER, flywheel.SPEAKER));
    second.rightBumper().onTrue(c_climber.moveTo(climber.FULLEXTENSION));
    second.leftBumper().onTrue(c_climber.moveTo(new Rotation2d[]{Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)}));
    second.rightTrigger(0.3).whileTrue(intake.Out());
    second.a().whileTrue(intake.In());
    second.x().whileTrue(intake.moveTo(Constants.Swerve.intake.IDLE, false));
    second.b().whileTrue(intake.moveTo(Constants.Swerve.intake.INTAKE, true));
    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    
    //parse string into a array of characters
    char[] autonSequence = Preferences.getString(Constants.AutoConstants.kAutoKey, Constants.AutoConstants.defaultAuto).toCharArray();
    SequentialCommandGroup auto = new SequentialCommandGroup(fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), intake.Out().withTimeout(0.2), intake.Stop().withTimeout(0.00001));
    char start = 'M';
    for (char step : autonSequence){
      if (step == 'T'){start = 'T';}
      else if(step == 'B'){start = 'B';}
      else if(step == '1'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.0001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testMid1-11")),
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testMid1-2")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER).withTimeout(1), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardCloseTop")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardCloseTop")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        
      }
      else if(step == '2'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){ //Done
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("midPath1")),
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("midPath2")),
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER).withTimeout(1), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardCloseMiddle")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardCloseMiddle")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
      else if(step == '3'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testMid3-1")),
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testMid3-2")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardCloseBottom")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardCloseBottom")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
      else if(step == '4'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M4Forward")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M4Backward")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardFarTopTop")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardTopTop")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
      else if(step == '5'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M5Forward")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M5Backward")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardFarTopCenter")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardFarTopCenter")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
      else if(step == '6'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M6Forward")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M6Backward")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardFarCenterCenter")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardFarCenterCenter")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
      else if(step == '7'){
        if (start == 'T'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M7Forward")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M7Backward")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testMid1-1")), //TODO
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testMid1-2")), //TODO
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardFarBottomCenter")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardFarBottomCenter")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
      else if(step == '8'){
        if (start == 'T'){ 
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-1")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("testTop1-2")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'M'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M8Forward")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("M8Backward")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
        if (start == 'B'){
          auto.addCommands(
            intake.moveTo(Constants.Swerve.intake.INTAKE, true), 
            intake.In().withTimeout(0.00001), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3ForwardFarBottomBottom")), 
            intake.Stop().withTimeout(0.00001), 
            intake.moveTo(Constants.Swerve.intake.IDLE, false), 
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("3BackwardFarBottomBottom")), 
            fwheel.moveTill(flywheel.SPEAKER, flywheel.SPEAKER), 
            intake.Out().withTimeout(0.2), 
            intake.Stop().withTimeout(0.00001));
        }
      }
    }
    if (start == 'M'){
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      s_Swerve.resetOdometry(new Pose2d(1.21, 5.56, new Rotation2d()));}
      else{
      s_Swerve.resetOdometry(new Pose2d(15.35, 5.56, Rotation2d.fromDegrees(180)));
      }
    }
    return auto;
    //return autoChooser.getSelected();
  }




  // Import pathplanner paths 
  public Command testPath(){return new PathPlannerAuto("testAuto");}
  // Left side autos
  public Command Left1Note(){return new PathPlannerAuto("Left1Note");}
  public Command Left2Note(){return new PathPlannerAuto("Left2Note");}
  // Mid side autos
  public Command Mid1Note(){return new PathPlannerAuto("Mid1Note");}
  public Command Mid2Note(){return new PathPlannerAuto("Mid2Note");}
  //Right side autos
  public Command Right1Note(){return new PathPlannerAuto("Right1Note");}
}
