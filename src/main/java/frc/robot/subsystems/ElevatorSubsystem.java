package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ElevatorSubsystem extends SubsystemBase {

    private final double EncoderL0 = 1.1;
    private final double EncoderL1 = 2.2;
    private final double EncoderL2 = 3.3;
    private final double EncoderL3 = 4.4;
    private final double EncoderL4 = 5.5;
    private final double maxHeight = 2.0;
    private final double minHeight = -2.0;
    private long t = System.nanoTime();
    private long pt = System.nanoTime();
    private RelativeEncoder encoder;

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
    private double krot = 0.0; // rotations/meter
    // private ProfiledPIDController elevatorPID = new ProfiledPIDController(kp, ki,
    // kd, new Constraints(maxV, maxA));
    private final static double upSpeed = 0.5;
    private final static double downSpeed = 0.1;
    private double ks = 0, kg = 0, kv = 0, ka = 0;
    // private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(ks, kg, kv);

    private double currentMaxVel = maxV;
    private TrapezoidProfile.Constraints ffc = new TrapezoidProfile.Constraints(maxV, maxA);
    private TrapezoidProfile.State ffState = new TrapezoidProfile.State();
    private TrapezoidProfile.State preRenfernce = new TrapezoidProfile.State();
    private TrapezoidProfile Profiler = new TrapezoidProfile(ffc);
    private ElevatorFeedforward ff = new ElevatorFeedforward(ks, kg, kv);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

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

        encoder = motorL.getEncoder();
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
        ffState.position = preRenfernce.position;
        ffState.velocity = 0.0;

        motorL.stopMotor();
        motorR.stopMotor();
    }

    public void runControlLoop() {
        double ffValue = 0.0;
        // TODO: change limits
        boolean running = false;
        preRenfernce.velocity = MathUtil.clamp(preRenfernce.velocity, -currentMaxVel, currentMaxVel);
        preRenfernce = Profiler.calculate(pt, preRenfernce, ffState);
        ffValue = ff.calculate(MathUtil.clamp(preRenfernce.velocity, -currentMaxVel, currentMaxVel));

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

    public Command setPos(DoubleSupplier height) {
        return Commands.runOnce(() -> {
            currentMaxVel = maxV;
            ffState.position = height.getAsDouble();
            ffState.velocity = 0.0;
        });

    }

    public Command setSpeed(DoubleSupplier velo) {
        return Commands.run(() -> {
            double tol = 0.1;
            currentMaxVel = MathUtil.clamp(velo.getAsDouble(), -maxV, maxV);
            if (Math.abs(velo.getAsDouble()) > tol) {
                if (velo.getAsDouble() > 0) {
                    ffState.position = maxHeight;
                    ffState.velocity = 0.0;
                } else {
                    ffState.position = minHeight;
                    ffState.velocity = 0.0;
                }
            } else {
                ffState.position = preRenfernce.position;
                ffState.velocity = 0.0;
            }

        });
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
        return this.runOnce(() -> new PrintCommand("pls put somthing here later"));
    }

    public Command sysIDCommand(double quasiTimeout, double timeout, double dynamicTimeout) {
        return m_sysIdRoutine.quasistatic(Direction.kForward)
                .withTimeout(quasiTimeout).onlyWhile(() -> {
                    return (checkLimtis() != LimitSwitchTrigger.TOP);
                })
                .andThen(Commands.waitSeconds(timeout))
                .andThen(m_sysIdRoutine
                        .quasistatic(Direction.kReverse).onlyWhile(() -> {
                            return (checkLimtis() != LimitSwitchTrigger.BOTTOM);
                        }))
                .withTimeout(quasiTimeout)
                .andThen(Commands.waitSeconds(timeout))
                .andThen(m_sysIdRoutine.dynamic(Direction.kForward)
                        .withTimeout(dynamicTimeout).onlyWhile(() -> {
                            return (checkLimtis() != LimitSwitchTrigger.TOP);
                        }))
                .andThen(Commands.waitSeconds(timeout))
                .andThen(m_sysIdRoutine.dynamic(Direction.kReverse)
                        .withTimeout(dynamicTimeout).onlyWhile(() -> {
                            return (checkLimtis() != LimitSwitchTrigger.BOTTOM);
                        }));

    }

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(

            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(null, null, null, null),
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to plumb the driving voltage to the motors.
                    voltage -> {
                        // if (checkLimtis() == LimitSwitchTrigger.TOP
                        // && Voltage.ofBaseUnits(0, Volts).compareTo(voltage) < 0) {
                        // motorL.setVoltage(voltage);
                        // motorR.setVoltage(voltage);
                        // }
                        // if (checkLimtis() == LimitSwitchTrigger.BOTTOM
                        // && Voltage.ofBaseUnits(0, Volts).compareTo(voltage) > 0) {
                        // motorL.setVoltage(voltage);
                        // motorR.setVoltage(voltage);
                        // }
                        // if (checkLimtis() == LimitSwitchTrigger.NONE) {
                        // motorL.setVoltage(voltage);
                        // motorR.setVoltage(voltage);
                        // }

                        motorL.setVoltage(voltage);
                        motorR.setVoltage(voltage);

                    },
                    // Tell SysId how to record a frame of data for each motor on the mechanism
                    // being
                    // characterized.
                    log -> {
                        // Record a frame for the left motors. Since these share an encoder, we consider
                        // the entire group to be one motor.
                        log.motor("elevator")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                motorL.get() * motorL.getBusVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(encoder.getPosition() / krot, Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(encoder.getVelocity() / 60.0 / krot, MetersPerSecond));
                    },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this));

    // public Command runElevator() {
    // return run(
    // () -> {
    // motorL.set(elevatorFF.calculate(elevatorPID.calculate(encoderL.get())));
    // motorR.set(elevatorFF.calculate(elevatorPID.calculate(encoderR.get())));
    // });
    // }
}
