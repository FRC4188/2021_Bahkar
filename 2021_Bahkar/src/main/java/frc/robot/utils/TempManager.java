package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TempManager {

    Drivetrain drivetrain;
    Shooter shooter;
    Turret turret;
    Hopper hopper;
    Intake intake;

    /**
     * Constructs a TempManaget object.
     * @param drivetrain Drivetrain subsytem.
     * @param shooter Shooter subsytem.
     * @param turret Turret subsystem.
     * @param hopper Hopper subsystem.
     * @param intake Intake subsystem.
     */
    public TempManager(Drivetrain drivetrain, Shooter shooter, Turret turret, Hopper hopper, Intake intake) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.turret = turret;
        this.hopper = hopper;
        this.intake = intake;

        SmartDashboard.putString("Temp Warnings", "None");
    }

    /**
     * Call in robotPeriodic to check all of the motor temperatures.
     */
    public void run() {
        StringBuilder sb = new StringBuilder();

        // check all the motors and append their temperature to the String Builder.
        if (drivetrain.getFrontLeftDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FLDrive: " + drivetrain.getFrontLeftDriveTemp() + ", ");
        if (drivetrain.getFrontLeftAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FLAngle: " + drivetrain.getFrontLeftAngleTemp() + ", ");
        if (drivetrain.getFrontRightDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FRDrive: " + drivetrain.getFrontRightDriveTemp() + ", ");
        if (drivetrain.getFrontRightAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FRAngle: " + drivetrain.getFrontRightAngleTemp() + ", ");
        if (drivetrain.getRearLeftDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RLDrive: " + drivetrain.getRearLeftDriveTemp() + ", ");
        if (drivetrain.getRearLeftAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RLAngle: " + drivetrain.getRearLeftAngleTemp() + ", ");
        if (drivetrain.getRearRightDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RRDrive: " + drivetrain.getRearRightDriveTemp() + ", ");
        if (drivetrain.getRearRightAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RRAngle: " + drivetrain.getRearRightAngleTemp() + ", ");
        if (shooter.getUpperTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("Upper Shooter: " + shooter.getUpperTemp() + ",");
        if (shooter.getLowerTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("Lower Shooter: " + shooter.getLowerTemp() + ",");
        if (turret.getTemperature() > Constants.Robot.FIVEFIFTY_MAX_TEMP) sb.append("Turret: " + turret.getTemperature());
        if (hopper.getTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("Hopper: " + hopper.getTemp() + ",");
        if (intake.getMotorTemp() > Constants.Robot.FIVEFIFTY_MAX_TEMP) sb.append("Intake: " + intake.getMotorTemp() + ",");

        if (sb.length() > 1) SmartDashboard.putString("Temp Warnings", sb.toString());
        else if (SmartDashboard.getString("Temp Warnings", "None") != "None") SmartDashboard.putString("Temp Warnings", "None");
    }
}