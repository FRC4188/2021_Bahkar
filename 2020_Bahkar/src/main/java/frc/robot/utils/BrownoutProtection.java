package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class BrownoutProtection {

    Drivetrain drivetrain;
    Turret turret;

    double min_volts = Constants.ROBOT_MIN_VOLTS;
    double mid_volts = Constants.ROBOT_MID_VOLTS;
    
    public BrownoutProtection(Drivetrain drivetrain, Turret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
    }

    public void run() {

    }
}