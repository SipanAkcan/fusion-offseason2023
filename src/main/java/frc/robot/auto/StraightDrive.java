package frc.robot.auto;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;

public class StraightDrive{

    private AutoConfigurer autoConfigurer;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    public StraightDrive(AutoConfigurer autoConfigurer, RelativeEncoder leftEncoder, RelativeEncoder rightEncoder){
        this.autoConfigurer = autoConfigurer;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    // Use this methods for PID calculation
    public double goXmeter(double meterSetpoint){       
        return calculate(Units.feetToMeters(getEncodersFeetAverage()), meterSetpoint);
    }

    public double goXfeet(double feetSetpoint){
       return calculate(getEncodersFeetAverage(), feetSetpoint);
    }

    // PID Controller
    public double calculate(double currentDistance, double setpoint){
        return autoConfigurer.getEncoderPID().calculate(currentDistance, setpoint);
    }

    // Use this methods for measurement informations
    public double getEncodersFeetAverage(){
        return (leftEncoder.getCountsPerRevolution()*AutoConstants.K_DRIVE_TICK_2_FEET + rightEncoder.getCountsPerRevolution()*AutoConstants.K_DRIVE_TICK_2_FEET)/2;
    }

    public double getLeftEncoderFeet(){
        return leftEncoder.getCountsPerRevolution()*AutoConstants.K_DRIVE_TICK_2_FEET;
    }

    public double getRightEncoderFeet(){
        return rightEncoder.getCountsPerRevolution()*AutoConstants.K_DRIVE_TICK_2_FEET;
    }
}