package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CSPMath;

public class Shooter extends SubsystemBase {

    private final WPI_TalonFX upperShooterMotor = new WPI_TalonFX(10);
    private final WPI_TalonFX lowerShooterMotor = new WPI_TalonFX(11);

    private Notifier shuffle;

    private Sensors sensors;
    
    //https://www.omnicalculator.com/physics/projectile-motion

    public Shooter(Sensors sensors) {
        motorInits();

        SmartDashboard.putNumber("Set Shooter Velocity", 0.0);
        SmartDashboard.putNumber("Set Shooter Power", 0.0);

        shuffle = new Notifier(() -> updateShuffleboard());
        shuffle.startPeriodic(0.1);

        this.sensors = sensors;
    }

    @Override
    public void periodic() {
    }

    public void motorInits() {
        lowerShooterMotor.config_kP(0, Constants.shooter.kP);
        lowerShooterMotor.config_kI(0, Constants.shooter.kI);
        lowerShooterMotor.config_kD(0, Constants.shooter.kD);
        lowerShooterMotor.config_kF(0, Constants.shooter.kF);

        lowerShooterMotor.setNeutralMode(NeutralMode.Coast);

        lowerShooterMotor.configClosedloopRamp(Constants.shooter.RAMP_RATE);
        lowerShooterMotor.configOpenloopRamp(Constants.shooter.RAMP_RATE);

        lowerShooterMotor.setInverted(true);
        upperShooterMotor.setInverted(InvertType.FollowMaster);

        lowerShooterMotor.configClosedloopRamp(Constants.shooter.RAMP_RATE);

        upperShooterMotor.follow(lowerShooterMotor);
    }

    private void updateShuffleboard() {
        SmartDashboard.putNumber("Shooter Speed", getLowerVelocity());
        SmartDashboard.putNumber("Shooter Voltage", lowerShooterMotor.get() * RobotController.getInputVoltage());
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        lowerShooterMotor.config_kP(0, kP);
        lowerShooterMotor.config_kI(0, kI);
        lowerShooterMotor.config_kD(0, kD);
        lowerShooterMotor.config_kF(0, kF);
    }

    public void closeNotifier() {
        shuffle.close();
    }

    public void openNotifier() {
        shuffle.startPeriodic(0.1);
      }

    /**
     * Sets shooter motors to a given percentage [-1.0, 1.0].
     */
    public void setPercentage(double percent) {
        lowerShooterMotor.set(percent);
    }

    /**
     * Sets shooter motors to a given velocity in rpm.
     */
    public void setVelocity(double velocity) {
        lowerShooterMotor.set(ControlMode.Velocity, velocity);
    }

    public void setZoneVelocity() {
    }

    /**
     * Gets left shooter motor velocity in rpm.
     */
    public double getLowerVelocity() {
        return (lowerShooterMotor.getSelectedSensorVelocity() / 2048.0) * 600.0;
    }

    /**
     * Gets right shooter motor velocity in rpm.
     */
    public double getUpperVelocity() {
        return (upperShooterMotor.getSelectedSensorVelocity() / 2048.0) * 600.0;
    }

    /**
     * Returns left shooter motor temperature in Celcius.
     */
    public double getLowerTemp() {
        return lowerShooterMotor.getTemperature();
    }

    /**
     * Returns right shooter motor temperature in Celcius.
     */
    public double getUpperTemp() {
        return upperShooterMotor.getTemperature();
    }

    public boolean isReady() {
        return false; //Math.abs(getLowerVelocity() - Constants.Shooter.SHOOTING_VEL) < Constants.Shooter.SHOOTING_TOLERANCE;
    }
}