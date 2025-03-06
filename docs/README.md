# Docs

## Table of Contents
- [Bindings](#bindings)
- [CAN IDs](#can-ids)

## Bindings

### Teleop

#### Xbox Controller (Driver)
| Binding | Action | Description |
|---------|--------|-------------|
| Start Button | Zero gyro | Resets gyro heading |
| Left Bumper | Lock drive | Locks drivebase |
| Right Bumper | Intake | Runs intake |

#### Left Joystick (Operator) [`Port 1`]
| Binding | Action | Description |
|---------|--------|-------------|
| Button 5 | Coral station | Moves elevator and tilt to coral station height |
| Button 3 | L2 position | Moves elevator and tilt to L2 height |
| Button 4 | L3 position | Moves elevator and tilt to L3 height |
| Button 6 | L4 position | Moves elevator and tilt to L4 height |

### Test Mode


#### Xbox Controller (Driver)
| Binding | Action | Description |
|---------|--------|-------------|
| X Button | Fake vision | Adds fake vision reading |
| Back Button | Center modules | Centers swerve modules |
| B Button | Zero elevator | Zeros elevator position |
| Y Button | Drive to distance | Drives forward 1m at 0.2m/s |
| Left Bumper | Play music | Plays music |
| Right Bumper | Play music | Plays music |
| Right Trigger | Eject | Runs eject |

#### Left Joystick (Control 1) - Test Mode
| Binding | Action | Description |
|---------|--------|-------------|
| Trigger | Manual elevator | Controls elevator speed manually |
| Button 7 | Demo mode | Cycles through elevator positions |
| Button 8 | Climb | Controls climber up |
| Button 9 | Descend | Controls climber down |
| Button 10 | Zero tilt | Zeros tilt position |
| Button 11 | Tilt down | Nudges tilt down |
| Button 12 | Tilt up | Nudges tilt up |
| Button 2 | Tilt SysID | Runs tilt system identification |

#### Right Joystick (Operator) [`Port 2`]
| Binding | Action | Description |
|---------|--------|-------------|
| Button 9 | Elevator SysID | Runs elevator system identification |
| Button 10 | Tilt SysID | Runs tilt system identification |
| Button 11 | Angle motor SysID | Runs swerve angle motor system identification |
| Button 12 | Drive motor SysID | Runs swerve drive motor system identification |

## CAN IDs

- Drive Motor IDs:
  - driveMotorFR = 12
  - driveMotorFL = 13
  - driveMotorBR = 14
  - driveMotorBL = 15

- Turn Motor IDs:
  - turnMotorFR = 16
  - turnMotorFL = 17
  - turnMotorBR = 18 
  - turnMotorBL = 19

- CanCoder IDs:
  - canCoderFR = 20
  - canCoderRL = 21
  - canCoderBR = 22
  - canCoderBL = 23

- Scoring Motor IDs:
  - scoreMotor1 = 24
  - scoreMotor2 = 25 
  - scoreTiltMotor = 28

- Elevator Motor IDs:
  - elevatorMotorL = 26
  - elevatorMotorR = 27

- Climber Motor IDs:
  - beefyMotor = 29
