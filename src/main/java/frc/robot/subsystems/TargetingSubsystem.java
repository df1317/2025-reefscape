package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoScoring;
import frc.robot.libs.AllianceFlipUtil;
import frc.robot.libs.FieldConstants;
import frc.robot.libs.FieldConstants.Reef;
import frc.robot.libs.FieldConstants.ReefHeight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/* -----------
 * TargetingSubsystem
 * ---
 * instantiates a list of all the aliance relative reef branches and provides methods to target the closest reef branch
 * autoTargetCommand: targets the closest reef branch
 * driveToCoralTarget: drives to the coral target
 * driveToAlgaeTarget: drives to the algae target
 * getCoralTargetPose: gets the coral target pose
 * getAlgaeTargetPose: gets the algae target pose
 * autoTarget: targets the closest reef branch
 */
public class TargetingSubsystem extends SubsystemBase {

	private ReefBranch targetBranch;
	private Side targetSide;

	private List<Pose2d> reefBranches = null;
	private List<Pose2d> allianceRelativeReefBranches = null;
	private Map<Pose2d, ReefBranch> reefPoseToBranchMap = null;

	private void initializeBranchPoses() {
		reefBranches = new ArrayList<>();
		reefPoseToBranchMap = new HashMap<>();
		for (int branchPositionIndex = 0; branchPositionIndex < Reef.branchPositions.size(); branchPositionIndex++) {
			Map<ReefHeight, Pose3d> branchPosition = Reef.branchPositions.get(branchPositionIndex);
			Pose2d targetPose = branchPosition.get(ReefHeight.L4).toPose2d();
			reefBranches.add(targetPose);
			reefPoseToBranchMap.put(targetPose, ReefBranch.values()[branchPositionIndex]);
			reefPoseToBranchMap.put(AllianceFlipUtil.flip(targetPose), ReefBranch.values()[branchPositionIndex]);
		}
		allianceRelativeReefBranches = reefBranches.stream().map(AllianceFlipUtil::apply).collect(Collectors.toList());
	}

	public TargetingSubsystem() {
		new Trigger(() -> DriverStation.getAlliance().isPresent()).toggleOnTrue(
			Commands.runOnce(this::initializeBranchPoses)
		);
	}

	public Command setBranchCommand(ReefBranch branch) {
		return Commands.runOnce(() -> {
			targetBranch = branch;
		});
	}

	public ReefBranch getTargetBranch() {
		return targetBranch;
	}

	public Command driveToCoralTarget(SwerveSubsystem swerveDrive) {
		return driveToCoralTarget(swerveDrive, 0.6, 1);
	}

	public Command driveToCoralTarget(SwerveSubsystem swerveDrive, double speed, double acceleration) {
		return Commands.print("GOING TO POSE")
			.andThen(
				Commands.runOnce(() -> {
					System.out.println("drivetoCoralTarget");
					swerveDrive.getSwerveDrive().field.getObject("target").setPose(getCoralTargetPose());
				})
			)
			.andThen(swerveDrive.driveToPose(this::getCoralTargetPose, speed, acceleration))
			.andThen(Commands.print("DONE GOING TO POSE"));
	}

	public Pose2d getCoralTargetPose() {
		System.out.println("getCoarlTargetPose");
		Pose2d scoringPose = Pose2d.kZero;
		if (targetBranch != null) {
			Pose2d startingPose = Reef.branchPositions.get(targetBranch.ordinal()).get(ReefHeight.L2).toPose2d();
			SmartDashboard.putString("Targetted Coral Pose without Offset (Meters)", startingPose.toString());
			scoringPose = startingPose.plus(
				targetSide == Side.LEFT ? AutoScoring.Reef.coralOffsetL : AutoScoring.Reef.coralOffsetR
			);
			scoringPose = scoringPose.plus(getBranchSpecificOffset(targetBranch));
			SmartDashboard.putString("Targetted Coral Pose with Offset (Meters)", scoringPose.toString());
		} else {
			System.out.println("targetBranch = null");
		}
		return AllianceFlipUtil.apply(scoringPose);
	}

	public Pose2d autoTarget(Supplier<Pose2d> currentPose) {
		if (reefBranches == null) {
			initializeBranchPoses();
		}
		System.out.println("autoTarget");

		Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
		targetBranch = reefPoseToBranchMap.get(selectedTargetPose);
		targetSide = (targetBranch.ordinal() % 2 == 0) ? Side.LEFT : Side.RIGHT;
		return selectedTargetPose;
	}

	public Command autoTargetCommand(Supplier<Pose2d> currentPose) {
		return Commands.runOnce(() -> {
			autoTarget(currentPose);
			System.out.println("Auto-targetting complete - Selected: " + targetBranch.name());
		});
	}

	/* -----------
	 * Pair auto targeting
	 * ---
	 * getReefFromPairAndSide: gets the reef branch from the pair and side
	 */

	private ReefBranch getReefFromPairAndSide(ReefBranch pairBase, Side side) {
		int ordinal = pairBase.ordinal();
		// If it's an even number, it's the left side (A, C, E, etc.)
		// If it's an odd number, it's the right side (B, D, F, etc.)
		if (side == Side.LEFT && (ordinal % 2 == 1)) {
			return ReefBranch.values()[ordinal - 1];
		} else if (side == Side.RIGHT && (ordinal % 2 == 0)) {
			return ReefBranch.values()[ordinal + 1];
		}
		return pairBase;
	}

	public Pose2d autoTargetPair(Supplier<Pose2d> currentPose, Side preferredSide) {
		if (reefBranches == null) {
			initializeBranchPoses();
		}
		System.out.println("autoTaregtPair");

		// Find the closest reef position
		Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
		ReefBranch closestBranch = reefPoseToBranchMap.get(selectedTargetPose);

		// Get the base branch of the pair (always the left one)
		ReefBranch pairBase = ReefBranch.values()[closestBranch.ordinal() - (closestBranch.ordinal() % 2)];

		// Set the target branch based on the preferred side
		targetBranch = getReefFromPairAndSide(pairBase, preferredSide);
		targetSide = preferredSide;

		// Return the pose for the selected branch
		return allianceRelativeReefBranches.get(targetBranch.ordinal());
	}

	public Command autoTargetPairCommand(Supplier<Pose2d> currentPose, Side preferredSide) {
		return Commands.runOnce(() -> {
			autoTargetPair(currentPose, preferredSide);
			System.out.println("Auto-targetting pair complete - Selected: " + targetBranch.name());
		});
	}

	/* -----------
	 * areWeAllowedToDrive
	 * ---
	 * areWeAllowedToDrive: checks if we are allowed to drive
	 */

	public boolean areWeAllowedToDrive(Supplier<Pose2d> currentPose) {
		if (targetBranch == null) {
			return false;
		}

		Pose2d currentLocation = currentPose.get();
		Pose2d targetLocation = allianceRelativeReefBranches.get(targetBranch.ordinal());
		double distance = currentLocation.getTranslation().getDistance(targetLocation.getTranslation());

		System.out.println(distance);

		return distance < 2.5;
	}

	/* -----------
	 * Source Commands
	 * ---
	 * driveToSourceCommand: drives to the source
	 */

	public Command driveToSourceCommand(SwerveSubsystem swerveDrive) {
		return defer(() -> {
			Pose2d currentPose = swerveDrive.getPose();
			Pose2d sourcePose = currentPose
				.nearest(FieldConstants.CoralStation.bothPoses)
				.plus(
					new Transform2d(
						new Translation2d(Units.inchesToMeters(4), Units.inchesToMeters(8)),
						new Rotation2d(Units.degreesToRadians(180))
					)
				);

			return Commands.print("GOING TO SOURCE")
				.andThen(
					Commands.runOnce(() -> {
						swerveDrive.getSwerveDrive().field.getObject("target").setPose(sourcePose);
					})
				)
				.andThen(swerveDrive.driveToPose(() -> sourcePose, 0.6, 1))
				.andThen(Commands.print("DONE GOING TO SOURCE"));
		});
	}

	public Command driveToLeftBranch(SwerveSubsystem swerve) {
		return autoTargetPairCommand(swerve::getPose, Side.LEFT)
			.andThen(driveToCoralTarget(swerve))
			.andThen(Commands.print("ened the auto routine thing"));
	}

	public Command driveToRightBranch(SwerveSubsystem swerve) {
		return autoTargetPairCommand(swerve::getPose, Side.RIGHT)
			.andThen(driveToCoralTarget(swerve))
			.andThen(Commands.print("ened the auto routine thing"));
	}

	public Transform2d getBranchSpecificOffset(ReefBranch branch) {
		double x = 0; // in inches; positive is robot forward
		double y = 0; // in inches; positive is left
		double rot = 0; // in degrees; positive is counter clockwise

		switch (branch) {
			case I:
				y = -4;
				x = -4;
				rot = -5;
				break;
			default:
				x = 0;
				y = 0;
				rot = 0;
				break;
		}

		return new Transform2d(
			new Translation2d(Units.inchesToMeters(x), Units.inchesToMeters(y)),
			new Rotation2d(Units.degreesToRadians(rot))
		);
	}

	public enum Side {
		LEFT,
		RIGHT
	}

	public enum ReefBranch {
		A,
		B,
		K,
		L,
		I,
		J,
		G,
		H,
		E,
		F,
		C,
		D
	}
}
