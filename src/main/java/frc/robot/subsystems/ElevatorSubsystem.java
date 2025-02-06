package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final double EncoderL0 = 0.0;
    private final double EncoderL1 = 0.0;
    private final double EncoderL2 = 0.0;
    private final double EncoderL3 = 0.0;
    private final double EncoderL4 = 0.0;
    private long t = System.nanoTime();
    private long pt = System.nanoTime();

    private enum LimitSwitchTrigger {
        TOP,
        NONE,
        BOTTOM,
        TEST

    };

    private LimitSwitchTrigger state = LimitSwitchTrigger.TEST;
    private SparkMax motorL;
    private SparkMax motorR;
    private SparkClosedLoopController controllerL;
    private SparkClosedLoopController controllerR;
    private SparkMaxConfig config = new SparkMaxConfig();
    // private DutyCycleEncoder encoderL = new DutyCycleEncoder(2);
    // private DutyCycleEncoder encoderR = new DutyCycleEncoder(3);
    private double kp = 0.1, ki = 0, kd = 0;
    private double maxV = 1, maxA = 1;
    private double krot = 0.0;
    private ProfiledPIDController elevatorPID = new ProfiledPIDController(kp, ki, kd, new Constraints(maxV, maxA));
    private final static double upSpeed = 0.5;
    private final static double downSpeed = 0.1;
    private double ks = 0, kg = 0, kv = 0, ka = 0;
    private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(ks, kg, kv);

    private TrapezoidProfile.Constraints ffc = new TrapezoidProfile.Constraints(maxV, maxA);
    private TrapezoidProfile.State ffState = new TrapezoidProfile.State();
    private TrapezoidProfile.State preRenfernce = new TrapezoidProfile.State();
    private TrapezoidProfile profiler = new TrapezoidProfile(ffc);
    private ElevatorFeedforward ff = new ElevatorFeedforward(ks, kg, kv);

    public ElevatorSubsystem() {
        motorL = new SparkMax(26, MotorType.kBrushless);
        motorR = new SparkMax(27, MotorType.kBrushless);
        // the defualt is lsot zero
        config.closedLoop
                .p(kp, ClosedLoopSlot.kSlot0)
                .i(ki, ClosedLoopSlot.kSlot0)
                .d(kd, ClosedLoopSlot.kSlot0);

        motorL.configure(config, null, null);
        motorR.configure(config, null, null);

        controllerL = motorL.getClosedLoopController();
        controllerR = motorR.getClosedLoopController();
    }

    public void setDesiredPosistion(double height, double time) {
        ffState.position = height;
        ffState.velocity = 0.0;
    }

    private LimitSwitchTrigger checkLimtis() {
        System.out.println("sorry you need limit switches!");
        return LimitSwitchTrigger.TEST;

    }

    private void motorBreak() {
        System.out.println("sorry you need to motor break!");
    }

    public void runControlLoop() {
        double ffValue = 0.0;
        // TODO: change limits
        boolean running = false;

        preRenfernce = profiler.calculate(pt, preRenfernce, ffState);
        ffValue = ff.calculate(preRenfernce.velocity);

        switch (checkLimtis()) {
            case NONE:
                running = true;

                break;
            case BOTTOM:
                this.motorBreak();
                if (preRenfernce.velocity > 0) {
                    running = true;
                }
                break;
            case TOP:
                this.motorBreak();
                if (preRenfernce.velocity < 0) {
                    running = true;
                }
                break;
            case TEST:
                System.out.println("why did you run this? you forgot to program in the limits");
                running = false;
                break;

        }
        if (running) {
            controllerL.setReference(preRenfernce.position * krot,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    ffValue);
            controllerR.setReference(preRenfernce.position * krot,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    ffValue);
        }
        pt = t;

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

            if (input > 0) {
                motorL.set(upSpeed * input);
                motorR.set(upSpeed * input);
            } else {
                motorL.set(downSpeed * input);
                motorR.set(downSpeed * input);
            }
        }, () -> {
            motorL.set(0);
            motorR.set(0);
        });

    }

    public Command setPointElevatorCommand(int height) {
        return this.runOnce(() -> new PrintCommand("pls put osmtu=hing here later"));
    }

    // public Command runElevator() {
    // return run(
    // () -> {
    // motorL.set(elevatorFF.calculate(elevatorPID.calculate(encoderL.get())));
    // motorR.set(elevatorFF.calculate(elevatorPID.calculate(encoderR.get())));
    // });
    // }
}
