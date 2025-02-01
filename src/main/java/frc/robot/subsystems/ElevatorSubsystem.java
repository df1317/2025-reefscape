package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax motor1;
    private SparkMax motor2;

    public ElevatorSubsystem() {
        motor1 = new SparkMax(26, MotorType.kBrushless);
        motor2 = new SparkMax(27, MotorType.kBrushless);
    }

    public Command raiseElevatorCommand(double input) {
        return this.startEnd(() -> {
            motor1.set(input);
            motor2.set(input);
        }, () -> {
            motor1.set(0);
            motor2.set(0);
        });

    }
}
