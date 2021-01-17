/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.commands.drive.test.WheelRotationTest;
import frc.robot.commands.drive.test.setPIDs;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.turret.FollowTarget;
import frc.robot.commands.turret.TurretToOneEighty;
import frc.robot.commands.turret.TurretToZero;
import frc.robot.commands.turret.ZeroTurret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Turret;
import frc.robot.utils.BrownoutProtection;
import frc.robot.utils.ButtonBox;
import frc.robot.utils.CspController;
import frc.robot.utils.CspSequentialCommandGroup;
import frc.robot.utils.TempManager;
import frc.robot.utils.trajectory.CircleTest;
import frc.robot.utils.trajectory.TestFile;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Sensors sensors = new Sensors();
  private Drivetrain drivetrain = new Drivetrain(sensors);
  private Turret turret = new Turret(sensors);

  private TempManager tempManager = new TempManager(drivetrain, turret);
  private BrownoutProtection bop = new BrownoutProtection(drivetrain, turret);

  CspController pilot = new CspController(0);
  CspController copilot = new CspController(1);
  ButtonBox bBox = new ButtonBox(2);
  
  // auto chooser initialization
  private final SendableChooser<CspSequentialCommandGroup> autoChooser =
    new SendableChooser<CspSequentialCommandGroup>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setDefaultCommands();
    configureButtonBindings();
    putChooser();

    SmartDashboard.putBoolean("Manual Drive", false);
  }

  public TempManager getTempManager() {
    return tempManager;
  }

  public BrownoutProtection getBrownoutProtection() {
    return bop;
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(pilot.getY(Hand.kLeft), pilot.getX(Hand.kLeft), pilot.getX(Hand.kRight), pilot.getBumper(Hand.kRight)), drivetrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    pilot.getBackButtonObj().whenPressed(new ResetGyro(sensors));
    pilot.getRbButtonObj().whileHeld(new FollowTarget(turret, true));
    pilot.getRbButtonObj().whenReleased(new FollowTarget(turret, false));
    pilot.getXButtonObj().whenPressed(new setPIDs(drivetrain));

    //copilot.getStartButtonObj().whenPressed(new ZeroTurret(turret));

    //bBox.getButton1Obj().whenPressed(new TurretToZero(turret));
    //bBox.getButton2Obj().whenPressed(new TurretToOneEighty(turret));
    //bBox.getButton3Obj().whenPressed(new SpontaneousToShoot(drivetrain, sensors));
  }

  private void putChooser() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.getSelected();

    return new FollowTrajectory(drivetrain, new CircleTest(drivetrain).getTrajectory(), true).getCommand();
  }
}
