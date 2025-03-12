package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoScoring;
import frc.robot.libs.AllianceFlipUtil;
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
public class TargetingSubsystem {

	private ReefBranch targetBranch;

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
		return Commands.print("GOING TO POSE")
			.andThen(
				Commands.runOnce(() -> {
					swerveDrive.getSwerveDrive().field.getObject("target").setPose(getCoralTargetPose());
				})
			)
			.andThen(swerveDrive.driveToPose(this::getCoralTargetPose))
			.andThen(Commands.print("DONE GOING TO POSE"));
	}

	public Pose2d getCoralTargetPose() {
		Pose2d scoringPose = Pose2d.kZero;
		if (targetBranch != null) {
			Pose2d startingPose = Reef.branchPositions.get(targetBranch.ordinal()).get(ReefHeight.L2).toPose2d();
			SmartDashboard.putString("Targetted Coral Pose without Offset (Meters)", startingPose.toString());
			scoringPose = startingPose.plus(AutoScoring.Reef.coralOffset);
			SmartDashboard.putString("Targetted Coral Pose with Offset (Meters)", scoringPose.toString());
		}
		return AllianceFlipUtil.apply(scoringPose);
	}

	public Pose2d autoTarget(Supplier<Pose2d> currentPose) {
		if (reefBranches == null) {
			initializeBranchPoses();
		}

		Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
		targetBranch = reefPoseToBranchMap.get(selectedTargetPose);
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

		// Find the closest reef position
		Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
		ReefBranch closestBranch = reefPoseToBranchMap.get(selectedTargetPose);

		// Get the base branch of the pair (always the left one)
		ReefBranch pairBase = ReefBranch.values()[closestBranch.ordinal() - (closestBranch.ordinal() % 2)];

		// Set the target branch based on the preferred side
		targetBranch = getReefFromPairAndSide(pairBase, preferredSide);

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

		return distance < 1.5;
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
