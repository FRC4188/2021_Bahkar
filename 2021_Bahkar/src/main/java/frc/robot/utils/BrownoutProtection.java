package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class BrownoutProtection {

    Drivetrain drivetrain;

    double min_volts = Constants.RobotSpecs.MIN_VOLTS;
    double mid_volts = Constants.RobotSpecs.MID_VOLTS;
    
    public BrownoutProtection(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void run() {
    }
}