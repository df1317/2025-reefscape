package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {

    private SparkMax motor1;
    private SparkMax motor2;
    public DigitalInput coralSensor;

    public ScoringSubsystem() {
        motor1 = new SparkMax(24, MotorType.kBrushless);
        motor2 = new SparkMax(25, MotorType.kBrushless);
        coralSensor = new DigitalInput(1);
    }

    public void runIntake() {
        motor1.set(1);
        motor2.set(1);
    }

    public void stopMotors() {
        motor1.set(0);
        motor2.set(0);
    }

    public void runOuttake() {
        motor1.set(-1);
        motor2.set(-1);
    }
}
