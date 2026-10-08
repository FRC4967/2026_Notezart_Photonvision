package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;

public class TemperatureControl {
    /** the FIRST defined temperature safety limit. in Celsius */
    private static final double temperatureLimit = 90;

    /**
     * runs the motor only if the motor is under safety regulation temperatures to prevent motor smoking.
     * @param currentMotor the motor to set
     * @param setPower the power to set the motor to from -1 to 1
     */
    public static void tempSet(SparkMax currentMotor, double setPower) {
        if (currentMotor.getMotorTemperature() < temperatureLimit) {
            currentMotor.set(setPower);
        } else {
            currentMotor.stopMotor();
            System.out.println("MOTOR " + currentMotor.getDeviceId() + " IS TOO HOT TO RUN");
        }
    }

    public static void tempSetVoltage(SparkMax currentMotor, double setVoltage) {
        if (currentMotor.getMotorTemperature() < temperatureLimit) {
            currentMotor.setVoltage(setVoltage);
        } else {
            currentMotor.stopMotor();
            System.out.println("MOTOR " + currentMotor.getDeviceId() + " IS TOO HOT TO RUN");
        }
    }

    /**
     * runs the motor only if the motor is under safety regulation temperatures to prevent motor smoking.
     * @param currentMotor the motor to set
     * @param controlMode the control mode of the motor
     * @param setPower the power to set the motor to based on control mode
     */
    public static void tempSet(TalonSRX currentMotor, ControlMode controlMode, double setPower) {
        if (currentMotor.getTemperature() < temperatureLimit) {
            currentMotor.set(controlMode, setPower);
        } else {
            currentMotor.set(controlMode, 0);
            System.out.println("MOTOR" + currentMotor.getDeviceID() + " IS TOO HOT TO RUN");
        }
    }
}
