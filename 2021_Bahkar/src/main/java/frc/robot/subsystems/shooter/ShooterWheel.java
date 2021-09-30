package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class ShooterWheel {

    WPI_TalonFX slaveMotor;
    WPI_TalonFX masterMotor;

    public ShooterWheel(int upperCanID, int LowerCanID) {
        slaveMotor = new WPI_TalonFX(upperCanID);
        masterMotor = new WPI_TalonFX(LowerCanID);

        slaveMotor.configFactoryDefault();
        masterMotor.configFactoryDefault();

        masterMotor.config_kP(0, Constants.shooter.kP);
        masterMotor.config_kI(0, Constants.shooter.kI);
        masterMotor.config_kD(0, Constants.shooter.kD);
        masterMotor.config_kF(0, Constants.shooter.kF);

        masterMotor.setNeutralMode(NeutralMode.Coast);

        masterMotor.configClosedloopRamp(Constants.shooter.RAMP_RATE);
        masterMotor.configOpenloopRamp(Constants.shooter.RAMP_RATE);

        masterMotor.setInverted(true);
        slaveMotor.setInverted(InvertType.FollowMaster);

        masterMotor.configClosedloopRamp(Constants.shooter.RAMP_RATE);

        slaveMotor.follow(masterMotor);
    }

    public void setPower(double power) {
        masterMotor.set(power);
    }

    public void setVelocity(double velocity) {
        masterMotor.set(ControlMode.Velocity, velocity * Constants.robot.FALCON_ENCODER_TICKS / 600.0);
    }

    public double getPower() {
        return masterMotor.get();
    }

    public double getVelocity() {
        return masterMotor.getSelectedSensorVelocity() * 600.0 / Constants.robot.FALCON_ENCODER_TICKS;
    }

    public double getLowerTemp() {
        return masterMotor.getTemperature();
    }

    public double getUpperTemp() {
        return slaveMotor.getTemperature();
    }

    public boolean matchingVels() {
        return masterMotor.getSelectedSensorVelocity() == slaveMotor.getSelectedSensorVelocity();
    }
}