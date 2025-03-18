<h3 align="center">
    <img src="https://www.firstinspires.org/sites/default/files/2024-banner/frc_reefscape.gif" width="200" alt="Logo"/><br/>
    <span>2025 Reefscape</span>
    <img src="https://raw.githubusercontent.com/taciturnaxolotl/carriage/main/.github/images/transparent.png" height="30" width="0px"/>
</h3>

<p align="center">
    <i>It wasn't the water game we hoped for but its the water game we got.</i>
</p>

<p align="center">
	<img src="https://raw.githubusercontent.com/taciturnaxolotl/carriage/main/.github/images/line-break-thin.svg" />
</p>

> [!NOTE]
> the bindings and can ids are documented in [`docs`](/docs)

## What's this?

This is the code for 1317's 2025 robot for this years Reefscape FRC game!

<p align="center">
    <a href="https://www.youtube.com/watch?v=YWbxcjlY9JY">
        <img src="https://raw.githubusercontent.com/df1317/2025-reefscape/main/.github/images/reefscape.webp" alt="reefscape game animation cover image"/>
    </a>
</p>

## Contributing / Code Style

### Formatting

> [!WARNING]
> Make sure to do these instructions before you start working on the code. (Looking at you Holden and Rebekah 👀)

This project uses [jhipster/prettier-java](https://github.com/jhipster/prettier-java) to format the code. Assuming you already have `bun` installed, you can run the following command to format all the code in the project:

```bash
bun format
```

If you don't have `bun` installed then install it by running one of the following commands:

> linux / macos
```bash
curl -fsSL https://bun.sh/install | bash
```

> windows
```powershell
powershell -c "irm bun.sh/install.ps1 | iex"
```

Pretty please make sure you have prettier properly setup in vscode 🥺 (or your editor of choice; zed will just automatically work). The `.vscode/settings.json` file should automatically configure vscode to use prettier for formatting java files but double check to make sure.

You will also need to install the java extension for prettier by running the following command in this repo:

```bash
bun install
```

### Commit Messages

Please follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) standard for commit messages. This will help us keep track of changes and make it easier to generate changelogs. You are welcome to be funny in your commit message but please make sure the commit type is correct.

Here are some examples of commit messages that are and aren't acceptable:

```plaintext
✅ feat: add new feature
✅ bug: fixed the photonvision latency bug 🐞
⛔ added new feature
⛔ fixed smthing
✅ docs: update readme with commit warnings
⛔ updated readme :doc
😵 fixed elevator limits and weired negitive velocity :bug (ha take the mr k i did the fnacy commits)
✅ chore: ran prettier on the code for the 11 millionth time
```

### Branches

Make branches early and often! If you are working on a new feature that isn't directly reliant on a side branch then make a new branch and open a PR as quick as possible. Be wary of working on too many branches at once though; it can get confusing. Aim to merge the branch once you have verified that the code works on the robot.

### Commands and Subsystems

As much as possible try to use the WPILIB built-ins for commands and subsystems. WPILib has some awesome docs on [Commands](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands.html) and [Command Compositions](https://docs.wpilib.org/en/latest/docs/software/commandbased/command-compositions.html) as well as [Subsystems](https://docs.wpilib.org/en/latest/docs/software/commandbased/subsystems.html).

Here is a quick example of a subsystem with a command and then a command binding to a button:

```java
public class ExampleSubsystem extends SubsystemBase {
    private final SparkMotor motor;
    
    public ExampleSubsystem() {
        motor = new SparkMotor(0);
    }
    
    public Command getExampleCommand(BooleanSupplier speed) {
        return new RunCommand(() -> motor.set(speed.getAsDouble()), this);
    }
}
```

```java
public class RobotContainer {
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final Joystick joystick = new Joystick(0);
    
    public RobotContainer() {
        joystick.trigger().onTrue(elevatorSubsystem.setPos(() -> 0.3));
    }
}
```

## Why AGPL 3.0 as a license?

This is FIRST; our mission as teams is to learn and the best way to learn is to
share info with other teams!
<img src="https://cachet.dunkirk.sh/emojis/kitty-gun/r" align="right" /> The
AGPL 3.0 license only requires you to make your source code public if you are
distributing the software to others (e.g. selling it, or making it available for
download as a compiled project). If you are running this code on your robot or
forking this repo then you should be completely fine. Just don't ever charge for
this software. Ever. (Not sure why you would be charging for frc code though?)

<p align="center">
	<img src="https://raw.githubusercontent.com/taciturnaxolotl/carriage/main/.github/images/line-break.svg" />
</p>

<p align="center">
	<code>&copy 2025-present <a href="https://github.com/df1317">Digital Fusion FRC team 1317</a></code>
</p>

<p align="center">
	<a href="https://github.com/df1317/2025-reefscape/blob/main/LICENSE.md"><img src="https://img.shields.io/static/v1.svg?style=for-the-badge&label=License&message=AGPL 3.0&logoColor=d9e0ee&colorA=363a4f&colorB=b7bdf8"/></a>
</p>
