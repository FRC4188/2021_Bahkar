/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ClearDeadzone;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.auto.SixBall;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.hood.DashPosition;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.turret.TurretPower;
import frc.robot.commands.turret.TurretToOneEighty;
import frc.robot.commands.turret.TurretToZero;
import frc.robot.commands.turret.ZeroTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.BrownoutProtection;
import frc.robot.utils.components.ButtonBox;
import frc.robot.utils.components.CSPJoystick;
import frc.robot.utils.components.CspController;
import frc.robot.utils.components.LEDPanel;
import frc.robot.utils.components.CspController.Scaling;
import frc.robot.utils.TempManager;

/**
 * Controls the robot, holding commands, button bindings, and auto routines.
 */
public class RobotContainer {

  // Create the subsystems; Sensors first to be fed into each of the others.
  private Sensors sensors;
  private Drivetrain drivetrain;
  private Turret turret;
  private Hopper hopper;
  private Intake intake;
  private Shooter shooter;
  private Hood hood;
  private Climber climber;
  // private LEDPanel ledPanel;

  // Subsystem Regulation
  // private TempManager tempManager = new TempManager(drivetrain, shooter,
  // turret, hopper, intake);
  // private BrownoutProtection bop = new BrownoutProtection(drivetrain, turret);

  // Input devices
  CspController pilot = new CspController(0);
  CspController copilot = new CspController(1);
  CSPJoystick stick = new CSPJoystick(3);
  ButtonBox bBox = new ButtonBox(2);

  // Auto chooser initialization
  private final SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(LEDPanel ledPanel) {
    ledPanel = new LEDPanel(2);
    sensors = new Sensors();
    drivetrain = new Drivetrain(sensors);
    turret = new Turret(sensors, drivetrain);
    hopper = new Hopper(sensors);
    intake = new Intake();
    shooter = new Shooter();
    hood = new Hood(sensors, drivetrain);
    climber = new Climber();

    setDefaultCommands();
    configureButtonBindings();
    putChooser();
  }

  /**
   * @return TempManager object.
   */
  public TempManager getTempManager() {
    return null; // tempManager;
  }

  /**
   * @return BrownoutProtection object.
   */
  public BrownoutProtection getBrownoutProtection() {
    return null; // bop;
  }

  /**
   * Ends Notifier threads which feed the NetworkTables.
   */
  public void closeNotifiers() {
    drivetrain.closeNotifier();
    hood.closeNotifier();
    sensors.closeNotifier();
    shooter.closeNotifier();
    turret.closeNotifier();
  }

  /**
   * Begins the Notifier threads which feed the NetworkTables.
   */
  public void openNotifiers() {
    drivetrain.openNotifier();
    hood.openNotifier();
    sensors.openNotifier();
    shooter.openNotifier();
    turret.openNotifier();
  }

  /**
   * Method which assigns default commands to different subsystems.
   */
  private void setDefaultCommands() {
    // Drivetrain manual drive command.
    drivetrain.setDefaultCommand(new RunCommand( () -> drivetrain.drive(
    stick.getYAxis() + pilot.getY(Hand.kLeft, Scaling.SQUARED),
    stick.getXAxis() + pilot.getX(Hand.kLeft, Scaling.SQUARED),
    stick.getRotation() + pilot.getX(Hand.kRight, Scaling.SQUARED),
    pilot.getBumper(Hand.kRight) || stick.getPOV() == 180),
    drivetrain));
    
    // Set motors to do nothing by default.
    turret.setDefaultCommand(new TurretPower(turret, 0.0));
    intake.setDefaultCommand(new SpinIntake(intake, 0.0));
    hopper.setDefaultCommand(new SpinHopper(hopper, 0.0));
  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    /*
    Pilot Commands
    */

    // Intake commands.
    pilot.getAButtonObj().whenPressed(new SpinIntake(intake, 0.5, true));
    pilot.getAButtonObj().whenReleased(new SpinIntake(intake, 0.0, false));
    pilot.getBButtonObj().whenPressed(new SpinIntake(intake, -0.5, true));
    pilot.getBButtonObj().whenReleased(new SpinIntake(intake, 0.0, false));

    //Hopper (shoot) commands.
    pilot.getXButtonObj().whenPressed(new SpinHopper(hopper, 1.0, true));
    pilot.getXButtonObj().whenReleased(new SpinHopper(hopper, 0.0, false));
    pilot.getYButtonObj().whenPressed(new SpinHopper(hopper, -1.0, true));
    pilot.getYButtonObj().whenPressed(new SpinHopper(hopper, 0.0, false));

    //Relative referenced intake command.
    pilot.getLbButtonObj().whenPressed(new InstantCommand(() -> intake.toggle(), intake));

    /*
    Joystick Commands
    */

    // Intake commands.
    stick.get3ButtonObj().whenPressed(new SpinIntake(intake, 0.5, true));
    stick.get3ButtonObj().whenReleased(new SpinIntake(intake, 0.0, true));
    stick.get4ButtonObj().whenPressed(new SpinIntake(intake, -0.5, true));
    stick.get4ButtonObj().whenReleased(new SpinIntake(intake, 0.0, true));

    // Hopper (shoot) commands.
    stick.getTriggerButtonObj().whenPressed(new SpinHopper(hopper, 1.0, true));
    stick.getTriggerButtonObj().whenReleased(new SpinHopper(hopper, 0.0, true));
    stick.get5ButtonObj().whenPressed(new SpinHopper(hopper, -1.0, true));
    stick.get5ButtonObj().whenReleased(new SpinHopper(hopper, 0.0, true));

    // Relative referenced intake command.
    stick.get6ButtonObj().whenPressed(new InstantCommand(() -> intake.toggle()));

    // Auto-Aim command.
    stick.get2ButtonObj().whileHeld(new RunCommand(() -> turret.trackTarget(true), turret));
    stick.get2ButtonObj().whenReleased(new InstantCommand(() -> turret.trackTarget(false), turret));


    /*
    Copilot commands follow:
    */

    // Manually turn the turret.
    copilot.getDpadLeftButtonObj().whenPressed(new TurretPower(turret, 0.5, true));
    copilot.getDpadLeftButtonObj().whenReleased(new TurretPower(turret, 0.0, false));
    copilot.getDpadRightButtonObj().whenPressed(new TurretPower(turret, -0.5, true));
    copilot.getDpadRightButtonObj().whenReleased(new TurretPower(turret, 0.0, false));

    // Absolute referenced intake commands.
    copilot.getDpadDownButtonObj().whenPressed(new InstantCommand(() -> intake.lower()));
    copilot.getDpadUpButtonObj().whenPressed(new InstantCommand(() -> intake.raise()));

    // Turret commands.
    copilot.getAButtonObj().whileHeld(new RunCommand(() -> turret.trackTarget(true), turret));
    copilot.getAButtonObj().whenReleased(new InstantCommand(() -> turret.trackTarget(false), turret));

    // Climber commands.
    copilot.getRbButtonObj().whenPressed(new InstantCommand(() -> climber.setLeft(0.2), climber));
    copilot.getRbButtonObj().whenReleased(new InstantCommand(() -> climber.setLeft(0.0), climber));
    copilot.getLbButtonObj().whenPressed(new InstantCommand(() -> climber.setLeft(-1.0), climber));
    copilot.getLbButtonObj().whenReleased(new InstantCommand(() -> climber.setLeft(0.0), climber));

    /*
    Button box commands follow:
    */

    // Turret commands.
    bBox.getButton1Obj().whenPressed(new TurretToZero(turret));
    bBox.getButton2Obj().whenPressed(new TurretToOneEighty(turret));

    // Intake command.
    bBox.getButton3Obj().whenPressed(new ClearDeadzone(intake));

    // NetworkTable commands.
    bBox.getButton4Obj().whenPressed(new InstantCommand(() -> openNotifiers()));
    bBox.getButton5Obj().whenPressed(new InstantCommand(() -> closeNotifiers()));

    /*
    SmartDashboard commands follow:
    */

    // Turret command.
    SmartDashboard.putData("Zero Turret", new ZeroTurret(turret));

    // Drivetrain command.
    SmartDashboard.putData("Zero Gyro", new ResetGyro(sensors));
    SmartDashboard.putData("Reset Pose", new ResetOdometry(drivetrain));

    // Shooter commands.
    SmartDashboard.putData("Set Velocity", new InstantCommand(() -> shooter.setVelocity(SmartDashboard.getNumber("Set Shooter Velocity", 0.0)), shooter));
    SmartDashboard.putData("Set Power", new InstantCommand(() -> shooter.setPercentage(SmartDashboard.getNumber("Set Shooter Power", 0.0)), shooter));


    //Hood command.
    SmartDashboard.putData("Set Hood", new DashPosition(hood));
  }

  private void putChooser() {
    autoChooser.addOption("Do Nothing", null);
    autoChooser.addOption("Six-Ball Auto", new SixBall(drivetrain, shooter, hopper, sensors, turret, hood));
  }

  public void resetRobot() {
    sensors.resetGyro();
    drivetrain.resetOdometry(new Pose2d());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.getSelected();

    return autoCommand;
  }
}
