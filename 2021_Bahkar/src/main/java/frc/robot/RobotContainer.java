// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.drive.CenterBall;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.sensors.ResetOdometry;
import frc.robot.commands.sensors.ResetTranslation;
import frc.robot.commands.turret.FollowTarget;
import frc.robot.commands.turret.TurretPower;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.CspController;
import frc.robot.utils.TempManager;
import frc.robot.utils.CspController.Scaling;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  Swerve drive = Swerve.getInstance();
  Shooter shooter = Shooter.getInstance();
  Intake intake = Intake.getInstace();
  Hopper hopper = Hopper.getInstance();
  Climber climber = Climber.getInstance();
  Turret turret = Turret.getInstance();
  Hood hood = Hood.getInstance();

  CspController pilot = new CspController(0);
  CspController copilot = new CspController(1);

  SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    setDefaultCommands();
    configureButtonBindings();
    addChooser();

    TempManager.openNotifier();
  }

  private void setDefaultCommands() {
    // Drivetrain manual drive command.
    drive.setDefaultCommand(new RunCommand( () -> drive.drive(
      pilot.getY(Hand.kLeft, Scaling.CUBED),
      pilot.getX(Hand.kLeft, Scaling.CUBED),
      pilot.getX(Hand.kRight, Scaling.CUBED),
      false),
    drive));
    
    // Set shooter speed to nothing.
    //shooter.setDefaultCommand(new ShooterVelocity(0.0));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

      // Intake commands.
      pilot.getAButtonObj().whenPressed(new AutoIntake(true))
        .whenReleased(new AutoIntake(false));
      pilot.getBButtonObj().whenPressed(new SpinIntake(-0.5, true))
        .whenReleased(new SpinIntake(0.0, false));
  
      //Hopper (shoot) commands.
      pilot.getXButtonObj().whenPressed(new AutoShoot(true))
        .whenReleased(new AutoShoot(false));
      pilot.getYButtonObj().whenPressed(new SpinHopper(-1.0, true))
        .whenReleased(new SpinHopper(0.0, false));

      pilot.getDpadUpButtonObj().whenPressed(new SetPosition(true));
      pilot.getDpadDownButtonObj().whenPressed(new SetPosition(false));

      pilot.getDpadLeftButtonObj().whenPressed(new InstantCommand(() -> climber.engagePneuBrake(true)));
      pilot.getDpadRightButtonObj().whenPressed(new InstantCommand(() -> climber.engagePneuBrake(false)));
  
      //Relative referenced intake command.
      pilot.getLbButtonObj().whenPressed(new InstantCommand(() -> intake.toggleRaised(), intake));

      pilot.getRbButtonObj().whenPressed(new CenterBall(
        () -> pilot.getY(Hand.kLeft, Scaling.CUBED),
        () -> pilot.getX(Hand.kLeft, Scaling.CUBED),
        true
        )).whenReleased(new CenterBall(false));
  
      /*
      Copilot commands follow:
      */
  
      // Manually turn the turret.
      copilot.getDpadLeftButtonObj().whenPressed(new TurretPower(0.5, true))
        .whenReleased(new TurretPower(0.0, false));
      copilot.getDpadRightButtonObj().whenPressed(new TurretPower(-0.5, true))
        .whenReleased(new TurretPower(0.0, false));

      /*copilot.getDpadLeftButtonObj().whenPressed(new InstantCommand(() -> climber.engagePneuBrake(true)));
      copilot.getDpadRightButtonObj().whenPressed(new InstantCommand(() -> climber.engagePneuBrake(false)));*/
  
      // Absolute referenced intake commands.
      copilot.getDpadDownButtonObj().whenPressed(new InstantCommand(() -> intake.setRaised(true)));
      copilot.getDpadUpButtonObj().whenPressed(new InstantCommand(() -> intake.setRaised(false)));
  
      // Turret commands.
      copilot.getAButtonObj().whileHeld(new RunCommand(() -> turret.trackTarget(true), turret))
        .whenReleased(new InstantCommand(() -> turret.trackTarget(false), turret));

      copilot.getXButtonObj().whenPressed(new InstantCommand(() -> climber.engagePneuBrake(!climber.getPneuBrake())));

      // Climber commands.
      copilot.getRbButtonObj().whenPressed(new Climb(0.5, true))
        .whenReleased(new Climb(0.0, false));
      copilot.getLbButtonObj().whenPressed(new Climb(-0.5, true))
        .whenReleased(new Climb(0.0, false));
  
      copilot.getBButtonObj().whenPressed(new Climb(-1.0, true))
        .whenReleased(new Climb(0.0, false));

    /* SmartDashboard Commands */
    SmartDashboard.putData("Reset Gyro", new ResetGyro());
    SmartDashboard.putData("Reset Pose", new ResetOdometry());
    SmartDashboard.putData("Reset Translation", new ResetTranslation());
    SmartDashboard.putData("Send Shooter Velocity",
        new InstantCommand(() -> shooter.setVelocity(SmartDashboard.getNumber("Set Shooter Velocity", 0.0)), shooter));
    SmartDashboard.putData("Send Shooter Power",
        new InstantCommand(() -> shooter.setVelocity(SmartDashboard.getNumber("Set Shooter Power", 0.0)), shooter));
    /*SmartDashboard.putData("Send Hood Distance (mm)",
        new InstantCommand(() -> hood.setPosition(SmartDashboard.getNumber("Set Hood Position (mm)", 0.0) / 100.0)));*/
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Nothing", null);
    //autoChooser.addOption("Six-Ball", new SixBall());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
