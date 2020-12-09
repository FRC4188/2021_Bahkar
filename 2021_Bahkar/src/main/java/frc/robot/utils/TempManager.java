package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TempManager {

    Drivetrain drivetrain;

    public TempManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void run() {
        StringBuilder sb = new StringBuilder();

        if (drivetrain.getFrontLeftDriveTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("FL Drive: " + drivetrain.getFrontLeftDriveTemp() + ", ");
        if (drivetrain.getFrontLeftAngleTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("FL Angle: " + drivetrain.getFrontLeftAngleTemp() + ", ");
        if (drivetrain.getFrontRightDriveTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("FR Drive: " + drivetrain.getFrontRightDriveTemp() + ", ");
        if (drivetrain.getFrontRightAngleTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("FR Angle: " + drivetrain.getFrontRightAngleTemp() + ", ");
        if (drivetrain.getRearLeftDriveTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("RL Drive: " + drivetrain.getRearLeftDriveTemp() + ", ");
        if (drivetrain.getRearLeftAngleTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("RL Angle: " + drivetrain.getRearLeftAngleTemp() + ", ");
        if (drivetrain.getRearRightDriveTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("RR Drive: " + drivetrain.getRearRightDriveTemp() + ", ");
        if (drivetrain.getRearRightAngleTemp() > Constants.RobotSpecs.FALCON_MAX_TEMP) sb.append("RR Angle: " + drivetrain.getRearRightAngleTemp() + ", ");

        SmartDashboard.putString("Temp Warnings", sb.toString());
    }
}