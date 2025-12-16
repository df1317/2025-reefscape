package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
  private SparkMax motorL;
  private SparkMax motorR;

  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;

  private SparkMaxConfig config = new SparkMaxConfig();

  private SparkClosedLoopController controllerL;
  private SparkClosedLoopController controllerR;
}
