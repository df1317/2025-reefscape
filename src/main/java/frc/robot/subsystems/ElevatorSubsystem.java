package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax motorL;
    private SparkMax motorR;
    private DutyCycleEncoder encoderL = new DutyCycleEncoder(2);
    private DutyCycleEncoder encoderR = new DutyCycleEncoder(3);
    private double kp = 0.1, ki = 0, kd = 0;
    private PIDController elevatorPID = new PIDController(kp, ki, kd);

    public ElevatorSubsystem() {
        motorL = new SparkMax(26, MotorType.kBrushless);
        motorR = new SparkMax(27, MotorType.kBrushless);
    }

    private double getDesiredPosistion(int height) {
        switch (height) {
            case 0:
                return 0;
            case 1:
                return 0;
            case 2:
                return 0;
            case 3:
                return 0;
            default:
                return 0;
        }

    }

    public Command manualElevatorCommand(double input) {
        return this.startEnd(() -> {
            motorL.set(input);
            motorR.set(input);
        }, () -> {
            motorL.set(0);
            motorR.set(0);
        });

    }

    public Command setPointElevatorCommand(int height) {
        return this.runEnd(() -> {
            elevatorPID.setSetpoint(getDesiredPosistion(height));
            motorL.set(elevatorPID.calculate(encoderL.get()));
            motorR.set(elevatorPID.calculate(encoderR.get()));
        }, () -> {
            motorL.set(0);
            motorR.set(0);
        });
    }
}
