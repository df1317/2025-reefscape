package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

    public Command runIntakeCommand(boolean out) {
        return this.startEnd(() -> {
            motor1.set(out ? 1 : -1);
            motor2.set(out ? 1 : -1);
        }, () -> {
            motor1.set(0);
            motor2.set(0);
        });
    }

    public BooleanSupplier getCoralSensorState = () -> coralSensor.get();
}
