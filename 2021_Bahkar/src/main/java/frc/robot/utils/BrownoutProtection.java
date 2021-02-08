package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

/**
 * Class to reduce power expenditure in response to low voltage.
 */
public class BrownoutProtection {

    Drivetrain drivetrain;
    Turret turret;

    double min_volts = Constants.Robot.MIN_VOLTS;
    double mid_volts = Constants.Robot.MID_VOLTS;
    
    public BrownoutProtection(Drivetrain drivetrain, Turret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
    }

    public void run() {
        // TODO: Write the actual code.
    }
}