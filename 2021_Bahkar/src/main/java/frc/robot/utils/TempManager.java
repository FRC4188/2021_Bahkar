package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TempManager {

    Drivetrain drivetrain;
    Turret turret;

    public TempManager(Drivetrain drivetrain, Turret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
    }

    public void run() {
        StringBuilder sb = new StringBuilder();

        if (drivetrain.getFrontLeftDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FLDrive: " + drivetrain.getFrontLeftDriveTemp() + ", ");
        if (drivetrain.getFrontLeftAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FLAngle: " + drivetrain.getFrontLeftAngleTemp() + ", ");
        if (drivetrain.getFrontRightDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FRDrive: " + drivetrain.getFrontRightDriveTemp() + ", ");
        if (drivetrain.getFrontRightAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("FRAngle: " + drivetrain.getFrontRightAngleTemp() + ", ");
        if (drivetrain.getRearLeftDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RLDrive: " + drivetrain.getRearLeftDriveTemp() + ", ");
        if (drivetrain.getRearLeftAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RLAngle: " + drivetrain.getRearLeftAngleTemp() + ", ");
        if (drivetrain.getRearRightDriveTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RRDrive: " + drivetrain.getRearRightDriveTemp() + ", ");
        if (drivetrain.getRearRightAngleTemp() > Constants.Robot.FALCON_MAX_TEMP) sb.append("RRAngle: " + drivetrain.getRearRightAngleTemp() + ", ");
        if (turret.getTemperature() > Constants.Robot.FIVEFIFTY_MAX_TEMP) sb.append("Turret: " + turret.getTemperature());

        SmartDashboard.putString("Temp Warnings", sb.toString());
    }
}