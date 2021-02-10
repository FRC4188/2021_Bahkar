package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final TalonFX upperShooterMotor = new TalonFX(10);
    private final TalonFX lowerShooterMotor = new TalonFX(11);
    
    //https://www.omnicalculator.com/physics/projectile-motion

    public Shooter() {
        controllerInit();

        // setup encoders
        upperShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        lowerShooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        SmartDashboard.putNumber("Set Shooter Velocity", 0.0);
    }

    @Override
    public void periodic() {

    }

    public void controllerInit() {
        lowerShooterMotor.config_kP(0, Constants.Shooter.kP, 10);
        lowerShooterMotor.config_kI(0, Constants.Shooter.kI, 10);
        lowerShooterMotor.config_kD(0, Constants.Shooter.kD, 10);

        lowerShooterMotor.setNeutralMode(NeutralMode.Coast);

        lowerShooterMotor.configClosedloopRamp(Constants.Shooter.RAMP_RATE);
        lowerShooterMotor.configOpenloopRamp(Constants.Shooter.RAMP_RATE);

        lowerShooterMotor.setInverted(true);
        upperShooterMotor.setInverted(InvertType.FollowMaster);

        upperShooterMotor.follow(lowerShooterMotor);
    }

    /**
     * Sets shooter motors to a given percentage [-1.0, 1.0].
     */
    public void setPercentage(double percent) {
        lowerShooterMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Sets shooter motors to a given velocity in rpm.
     */
    public void setVelocity(double velocity) {
        velocity *= (Constants.Robot.FALCON_ENCODER_TICKS) / 600;
        lowerShooterMotor.set(ControlMode.Velocity, velocity);
    }

    /**
     * Gets left shooter motor velocity in rpm.
     */
    public double getLowerVelocity() {
        return (lowerShooterMotor.getSelectedSensorVelocity() * 600) / Constants.Robot.FALCON_ENCODER_TICKS;
    }

    /**
     * Gets right shooter motor velocity in rpm.
     */
    public double getUpperVelocity() {
        return (upperShooterMotor.getSelectedSensorVelocity() * 600) / Constants.Robot.FALCON_ENCODER_TICKS;
        //check if formula is correct by running at max rpm and see if the returned velocity is equal to 6380 rpm
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
}