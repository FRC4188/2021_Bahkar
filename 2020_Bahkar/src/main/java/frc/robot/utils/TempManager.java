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

        if (drivetrain.getFrontLeftDriveTemp() > Constants.FALCON_MAX_TEMP) sb.append("FLDrive: " + drivetrain.getFrontLeftDriveTemp() + ", ");
        if (drivetrain.getFrontLeftAngleTemp() > Constants.FALCON_MAX_TEMP) sb.append("FLAngle: " + drivetrain.getFrontLeftAngleTemp() + ", ");
        if (drivetrain.getFrontRightDriveTemp() > Constants.FALCON_MAX_TEMP) sb.append("FRDrive: " + drivetrain.getFrontRightDriveTemp() + ", ");
        if (drivetrain.getFrontRightAngleTemp() > Constants.FALCON_MAX_TEMP) sb.append("FRAngle: " + drivetrain.getFrontRightAngleTemp() + ", ");
        if (drivetrain.getRearLeftDriveTemp() > Constants.FALCON_MAX_TEMP) sb.append("RLDrive: " + drivetrain.getRearLeftDriveTemp() + ", ");
        if (drivetrain.getRearLeftAngleTemp() > Constants.FALCON_MAX_TEMP) sb.append("RLAngle: " + drivetrain.getRearLeftAngleTemp() + ", ");
        if (drivetrain.getRearRightDriveTemp() > Constants.FALCON_MAX_TEMP) sb.append("RRDrive: " + drivetrain.getRearRightDriveTemp() + ", ");
        if (drivetrain.getRearRightAngleTemp() > Constants.FALCON_MAX_TEMP) sb.append("RRAngle: " + drivetrain.getRearRightAngleTemp() + ", ");
        if (turret.getTemperature() > Constants.FIVEFIFTY_MAX_TEMP) sb.append("Turret: " + turret.getTemperature());

        SmartDashboard.putString("Temp Warnings", sb.toString());
    }
}