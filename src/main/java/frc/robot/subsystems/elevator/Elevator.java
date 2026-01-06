package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.krot;
import static frc.robot.subsystems.elevator.ElevatorConstants.maxHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.maxV;
import static frc.robot.subsystems.elevator.ElevatorConstants.minHeight;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  private double currentMaxVel = maxV;
  private long t = System.nanoTime();

  public TrapezoidProfile.Constraints ffc =
      new TrapezoidProfile.Constraints(ElevatorConstants.maxV, ElevatorConstants.maxA);
  public TrapezoidProfile.State ffState = new TrapezoidProfile.State();
  public TrapezoidProfile.State preRenfernce = new TrapezoidProfile.State();
  public TrapezoidProfile Profiler = new TrapezoidProfile(ffc);
  public ElevatorFeedforward ff =
      new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);

  private enum Limits {
    TOP, NONE, BOTTOM, TEST
  }

  @Override
  public void periodic() {
    double ffValue = 0.0;
    boolean running = false;
    double height = io.getLeftEncoderPosition() / krot;

    ffState.position = MathUtil.clamp(ffState.position, 0, maxHeight);
    ffState.velocity = 0.0;

    preRenfernce.position =
        MathUtil.isNear(ffState.position, height, 0.1) ? preRenfernce.position : height;
    // preRenfernce.velocity = (encoderL.getVelocity() / krot) / 60.0;

    preRenfernce.velocity = MathUtil.clamp(preRenfernce.velocity, -currentMaxVel, currentMaxVel);
    preRenfernce.position = MathUtil.clamp(preRenfernce.position, 0, maxHeight);

    preRenfernce = Profiler.calculate((System.nanoTime() - t) / 1e9, preRenfernce, ffState);

    t = System.nanoTime();
    ffValue = ff.calculate(MathUtil.clamp(preRenfernce.velocity, -currentMaxVel, currentMaxVel));

    switch (checkLimits()) {
      case NONE:
        running = true;
        break;
      case BOTTOM:
        if (preRenfernce.velocity > 0) {
          running = true;
        } else {
          io.motorBreak();
        }
        break;
      case TOP:
        if (preRenfernce.velocity < 0) {
          running = true;
        } else {
          io.motorBreak();
        }
        break;
      case TEST:
        System.out.println("why did you run this? you forgot to program in the limits");
        running = false;
        break;
    }

    if (running) {
      io.setReference(preRenfernce.position * krot, ffValue);
    }
  }

  public Command setSpeed(DoubleSupplier velo) {
    return Commands.run(() -> {
      double tol = 0.1;
      currentMaxVel = Math.abs(MathUtil.clamp(velo.getAsDouble(), -maxV, maxV));
      if (currentMaxVel > tol) {
        if (velo.getAsDouble() > 0) {
          ffState.position = maxHeight;
          ffState.velocity = 0.0;
        } else if (velo.getAsDouble() < 0) {
          ffState.position = minHeight;
          ffState.velocity = 0.0;
        }
      } else {
        ffState.position = preRenfernce.position;
        ffState.velocity = 0.0;
      }
    }).finallyDo(() -> {
      ffState.position = preRenfernce.position;
      ffState.velocity = 0.0;
      currentMaxVel = maxV;
    });
  }

  public Command setPos(DoubleSupplier height) {
    return Commands.runOnce(() -> {
      currentMaxVel = maxV;
      ffState.position = height.getAsDouble();
      ffState.velocity = 0.0;
    });
  }

  private Limits checkLimits() {
    double height = io.getLeftEncoderPosition() / krot;
    if (height >= maxHeight) {
      return Limits.TOP;
    } else if (height <= minHeight) {
      return Limits.BOTTOM;
    } else {
      return Limits.NONE;
    }
  }

}
