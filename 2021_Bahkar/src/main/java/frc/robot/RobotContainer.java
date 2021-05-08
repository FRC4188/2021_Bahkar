/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.auto.SixBall;
import frc.robot.commands.climb.ThrottleClimb;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.drive.ToAngle;
import frc.robot.commands.drive.trajectorycontrol.HolonomicControl;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.hood.DashPosition;
import frc.robot.commands.hood.SetPosition;
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
import frc.robot.utils.trajectory.WaypointsList;
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

  DoubleSupplier throttle = () -> stick.getThrottle();

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
    /*stick.getYAxis() + */pilot.getY(Hand.kLeft, Scaling.SQUARED),
    /*stick.getXAxis() + */pilot.getX(Hand.kLeft, Scaling.SQUARED),
    /*stick.getRotation() + */pilot.getX(Hand.kRight, Scaling.SQUARED),
    pilot.getBumper(Hand.kRight)),// || stick.getPOV() == 180),
    drivetrain));

    turret.setDefaultCommand(new RunCommand(() -> turret.setAngle(0.0), turret));

    /*drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(
      pilot.getY(Hand.kRight, Scaling.SQUARED),
      pilot.getX(Hand.kRight, Scaling.SQUARED),
      -pilot.getAngle(Hand.kLeft),
      pilot.atZero(Hand.kLeft)),
      drivetrain));*/
    
    // Set motors to do nothing by default.
    turret.setDefaultCommand(new TurretPower(turret, 0.0));
    //intake.setDefaultCommand(new SpinIntake(intake, 0.0));
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
    pilot.getBButtonObj().whenPressed(new SpinIntake(intake, 0.75, true))
      .whenReleased(new AutoIntake(intake, hopper, sensors, false));
    pilot.getAButtonObj().whenPressed(new SpinIntake(intake, -0.5, true))
      .whenReleased(new SpinIntake(intake, 0.0, false));

    //Hopper (shoot) commands.
    //pilot.getYButtonObj().whenPressed(new AutoShoot(shooter, turret, hood, hopper, sensors, true, 3500.0))
      //.whenReleased(new AutoShoot(shooter, turret, hood, hopper, sensors, false, 0.0));
    pilot.getYButtonObj().whenPressed(new SpinHopper(hopper, 1.0))
    .whenReleased(new SpinHopper(hopper, 0.0));
    pilot.getXButtonObj().whenPressed(new SpinHopper(hopper, -0.5))
    .whenReleased(new SpinHopper(hopper, 0.0));

    pilot.getDpadUpButtonObj().whenPressed(new SpinShooter(shooter, 2050.0));
    pilot.getDpadDownButtonObj().whenPressed(new SpinShooter(shooter, 0.0));

    //Relative referenced intake command.
    pilot.getLbButtonObj().whenPressed(new InstantCommand((
      
    ) -> intake.toggle(), intake));

    /*
    Joystick Commands
    */

    // Intake commands.
    //stick.get3ButtonObj().whenPressed(new IntakeThrottleSpeed(intake, () -> stick.getRawAxis(2), true));
    //stick.get3ButtonObj().whenReleased(new IntakeThrottleSpeed(intake, () -> stick.getRawAxis(2), false));
    stick.get3ButtonObj().whenPressed(new AutoIntake(intake, hopper, sensors, true))
      .whenReleased(new AutoIntake(intake, hopper, sensors, false));

    //stick.get3ButtonObj().whenPressed(new SpinIntake(intake, 1.0, true));
    //stick.get3ButtonObj().whenReleased(new SpinIntake(intake, 0.0, false));

    stick.get4ButtonObj().whenPressed(new SpinIntake(intake, -0.85, true))
      .whenReleased(new SpinIntake(intake, 0.0, false));

    // Hopper (shoot) commands.
    //stick.getTriggerButtonObj().whenPressed(new AutoShoot(shooter, turret, hood, hopper, sensors, true, 3500.0))
      //.whenReleased(new AutoShoot(shooter, turret, hood, hopper, sensors, false, 3500.0));
    stick.getTriggerButtonObj().whenPressed(new SpinHopper(hopper, 1.0, true));
    stick.getTriggerButtonObj().whenReleased(new SpinHopper(hopper, 0.0, false));
    stick.get5ButtonObj().whenPressed(new SpinHopper(hopper, -1.0, true))
      .whenReleased(new SpinHopper(hopper, 0.0, true));

    // Relative referenced intake command.
    stick.get6ButtonObj().whenPressed(new InstantCommand(() -> intake.toggle()));

    // Auto-Aim command.
    stick.get2ButtonObj().whileHeld(new RunCommand(() -> turret.trackTarget(true), turret))
      .whenReleased(new InstantCommand(() -> turret.trackTarget(false), turret));


    /*
    Copilot commands follow:
    */

    // Manually turn the turret.
    copilot.getDpadLeftButtonObj().whenPressed(new TurretPower(turret, 0.5, true))
      .whenReleased(new TurretPower(turret, 0.0, false));
    copilot.getDpadRightButtonObj().whenPressed(new TurretPower(turret, -0.5, true))
      .whenReleased(new TurretPower(turret, 0.0, false));

    // Absolute referenced intake commands.
    //copilot.getDpadDownButtonObj().whenPressed(new InstantCommand(() -> intake.toggle()));
    //copilot.getDpadUpButtonObj().whenPressed(new InstantCommand(() -> intake.raise()));

    copilot.getDpadDownButtonObj().whenPressed(new SetPosition(hood, 0.2));
    copilot.getDpadUpButtonObj().whenPressed(new SetPosition(hood, 0.77));

    // Climber commands.
    copilot.getRbButtonObj().whenPressed(new RunCommand(() -> climber.set(0.2), climber))
      .whenReleased(new InstantCommand(() -> climber.set(0.0), climber));
    copilot.getLbButtonObj().whenPressed(new RunCommand(() -> climber.set(-0.2), climber))
      .whenReleased(new InstantCommand(() -> climber.set(0.0), climber));

    copilot.getBButtonObj().whenPressed(new RunCommand(() -> climber.set(-0.6), climber))
      .whenReleased(new InstantCommand(() -> climber.set(0.0), climber));

    copilot.getXButtonObj().whenPressed(new InstantCommand(() -> climber.togglePneuBrake(), climber));

    copilot.getYButtonObj().whenPressed(new InstantCommand(() -> intake.toggle(), intake));

    //copilot.getAButtonObj().whenPressed(new ToAngle(drivetrain, 180.0));

    
    /*
    Button box commands follow:\
    */

    // Turret commands.
    bBox.getButton1Obj().whenPressed(new TurretToZero(turret));
    bBox.getButton2Obj().whenPressed(new TurretToOneEighty(turret));

    // Intake command.
    //bBox.getButton3Obj().whenPressed(new ClearDeadzone(intake));

    // NetworkTable commands.
    bBox.getButton4Obj().whenPressed(new InstantCommand(() -> openNotifiers()));
    bBox.getButton5Obj().whenPressed(new InstantCommand(() -> closeNotifiers()));

    /*
    SmartDashboard commands follow:
    */

    // Turret command.
    SmartDashboard.putData("Zero Turret", new ZeroTurret(turret));

    // Drivetrain command.
    SmartDashboard.putData("Zero Gyro", new ResetGyro(sensors, drivetrain));
    SmartDashboard.putData("Reset Pose", new ResetOdometry(drivetrain));

    // Shooter commands.
    SmartDashboard.putData("Set Velocity", new InstantCommand(() -> shooter.setVelocity(SmartDashboard.getNumber("Set Shooter Velocity", 0.0)), shooter));
    SmartDashboard.putData("Set Power", new InstantCommand(() -> shooter.setPercentage(SmartDashboard.getNumber("Set Shooter Power", 0.0)), shooter));


    //Hood command.
    SmartDashboard.putData("Set Hood", new DashPosition(hood));
  }

  private void putChooser() {
    autoChooser.setDefaultOption("Do Nothing", null);
    autoChooser.addOption("Six-Ball Auto", new SixBall(drivetrain, shooter, hopper, intake, sensors, turret, hood));
    autoChooser.addOption("3 Meter Test", new SequentialCommandGroup(
      new HolonomicControl(drivetrain, WaypointsList.SixBall.DOWN, new Rotation2d(Math.PI)),
      new HolonomicControl(drivetrain, WaypointsList.SixBall.BACK, new Rotation2d(Math.PI))
    ));

    SmartDashboard.putData("Auto Chooser", autoChooser);
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

  public void startTele() {
    drivetrain.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
    sensors.resetGyro(
      sensors.getYaw() + 180);
  }
}
