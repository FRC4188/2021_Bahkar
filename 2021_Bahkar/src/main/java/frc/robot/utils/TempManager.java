package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.climber.Climber;

public class TempManager {

    private static Swerve drive = Swerve.getInstance();
    private static Intake intake = Intake.getInstace();
    private static Shooter shooter = Shooter.getInstance();
    private static Hopper hopper = Hopper.getInstance();
    private static Climber climber = Climber.getInstance();
    private static Turret turret = Turret.getInstance();

    private static Notifier shuffle = new Notifier(() -> run());

    public static void openNotifier() {
        shuffle.startPeriodic(1.0);
    }

    public static void closeNotifier() {
        shuffle.close();
    }

    private static void run() {
        ArrayList<String> report = new ArrayList<String>();

        if (drive.getFrontLeftAngleTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Front Left Angle");
        if (drive.getFrontRightAngleTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Front Right Angle");
        if (drive.getRearLeftAngleTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Rear Left Angle");
        if (drive.getRearRightAngleTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Rear Right Angle");
        if (drive.getFrontLeftDriveTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Front Left Drive");
        if (drive.getFrontRightDriveTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Front Right Drive");
        if (drive.getRearLeftDriveTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Rear Left Drive");
        if (drive.getRearRightDriveTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Rear Right Drive");
        if (intake.getTemperature() > Constants.robot.FIVEFIFTY_MAX_TEMP)
            report.add("Intake");
        if (hopper.getTemperature() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Hopper");
        if (shooter.getUpperTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Upper Shooter");
        if (shooter.getLowerTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Lower Shooter");
        if (climber.getFalconTemp() > Constants.robot.FALCON_MAX_TEMP)
            report.add("Climber Falcon");
        if (climber.getNeoTemp() > Constants.robot.NEO_MAX_TEMP)
            report.add("Climber Neo");
        if (turret.getTemp() > Constants.robot.FIVEFIFTY_MAX_TEMP)
            report.add("Turret Temp");

        String product = String.join(", ", report);
        SmartDashboard.putString("Motor Temp Warnings", product);
    }
}