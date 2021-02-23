/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.intake.ClearDeadzone;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoOuttake;
import frc.robot.commands.hood.DashAngle;
import frc.robot.commands.hood.DashPosition;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.shooter.DashShooterPower;
import frc.robot.commands.shooter.DashVelocity;
import frc.robot.commands.turret.TurretPower;
import frc.robot.commands.turret.TurretToOneEighty;
import frc.robot.commands.turret.TurretToZero;
import frc.robot.commands.turret.ZeroTurret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.BrownoutProtection;
import frc.robot.utils.components.ButtonBox;
import frc.robot.utils.components.CspController;
import frc.robot.utils.components.LEDPanel;
import frc.robot.utils.CspSequentialCommandGroup;
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
  // private LEDPanel ledPanel;

  // Subsystem Regulation
  // private TempManager tempManager = new TempManager(drivetrain, shooter,
  // turret, hopper, intake);
  // private BrownoutProtection bop = new BrownoutProtection(drivetrain, turret);

  // Input devices
  CspController pilot = new CspController(0);
  CspController copilot = new CspController(1);
  ButtonBox bBox = new ButtonBox(2);

  // Auto chooser initialization
  private final SendableChooser<CspSequentialCommandGroup> autoChooser = new SendableChooser<CspSequentialCommandGroup>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(LEDPanel ledPanel) {
    ledPanel = new LEDPanel(2);
    sensors = new Sensors(/*ledPanel*/);
    drivetrain = new Drivetrain(sensors);
    turret = new Turret(sensors, drivetrain);
    hopper = new Hopper(sensors);
    intake = new Intake();
    shooter = new Shooter(sensors);
    hood = new Hood(sensors, drivetrain);

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
    // drivetrain.closeNotifier();
    hood.closeNotifier();
    sensors.closeNotifier();
    shooter.closeNotifier();
    turret.closeNotifier();
  }

  /**
   * Begins the Notifier threads which feed the NetworkTables.
   */
  public void openNotifiers() {
    // drivetrain.openNotifier();
    hood.openNotifier();
    sensors.openNotifier();
    shooter.openNotifier();
    turret.openNotifier();
  }

  /**
   * Method which assigns default commands to different subsystems.
   */
  private void setDefaultCommands() {
    
    drivetrain.setDefaultCommand(new RunCommand( () -> drivetrain.drive(
    pilot.getY(Hand.kLeft), pilot.getX(Hand.kLeft), pilot.getX(Hand.kRight),
    pilot.getBumper(Hand.kRight)), drivetrain ));
     
    turret.setDefaultCommand(new TurretPower(turret, 0.0));
    intake.setDefaultCommand(new SpinIntake(intake, 0.0));
    hopper.setDefaultCommand(new SpinHopper(hopper, 0.0));
  }

  /**
   * Method which assigns commands to different button actions.
   */
  private void configureButtonBindings() {
    /*
    Pilot commands follow:
    */

    // Trigger the intaking and outtaking commands.
    pilot.getAButtonObj().whenPressed(new AutoIntake(intake, hopper, true));
    pilot.getAButtonObj().whenReleased(new AutoIntake(intake, hopper, false));
    pilot.getBButtonObj().whenPressed(new AutoOuttake(intake, hopper, true));
    pilot.getBButtonObj().whenReleased(new AutoOuttake(intake, hopper, false));

    // Relative referenced intake command.
    pilot.getRbButtonObj().whenPressed(new InstantCommand(() -> intake.toggle()));


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

    /*
    Button box commands follow:
    */

    // Turret commands.
    bBox.getButton1Obj().whenPressed(new TurretToZero(turret));
    bBox.getButton2Obj().whenPressed(new TurretToOneEighty(turret));

    // Intake command.
    bBox.getButton3Obj().whenPressed(new ClearDeadzone(intake));

    // NetworkTable commands.
    bBox.getButton4Obj().whenPressed(new RunCommand(() -> openNotifiers()));
    bBox.getButton5Obj().whenPressed(new RunCommand(() -> closeNotifiers()));

    /*
    SmartDashboard commands follow:
    */

    // Turret command.
    SmartDashboard.putData("Zero Turret", new ZeroTurret(turret));

    // Drivetrain command.
    SmartDashboard.putData("Zero Gyro", new ResetGyro(sensors));

    // Shooter commands.
    SmartDashboard.putData("Shooter PIDF", new InstantCommand(() -> shooter.setPIDF(
      SmartDashboard.getNumber("Shooter kP", 0.325),
      SmartDashboard.getNumber("Shooter kI", 0.0),
      SmartDashboard.getNumber("Shooter kD", 0.25),
      SmartDashboard.getNumber("Shooter kF", 0.0)
      )));
    SmartDashboard.putData("Set Velocity", new DashVelocity(shooter));


    //Hood command.
    SmartDashboard.putData("Set Hood", new DashPosition(hood));
  }

  private void putChooser() {
    autoChooser.addOption("Do Nothing", null);
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
