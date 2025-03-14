# Docs

## Table of Contents
- [Bindings](#bindings)  
- [CAN IDs](#can-ids)

## Bindings

### Teleop

#### Xbox Controller (Driver) [`Port 0`]
| Binding | Action | Description |
|---------|--------|-------------|
| A Button | Zero gyro | Resets gyro heading |
| X Button | Lock drive | Locks drivebase |
| Left Bumper | Lock drive | Locks drivebase |
| Right Bumper | play music | Kraken sings the duck song |

#### Left Joystick (Operator) [`Port 1`]
| Binding | Action | Description |
|---------|--------|-------------|
| Trigger | Auto Intake / Eject | based on coral sensor | 
| Trigger & Button 7 | Eject Coral | manual mode overrides coral sensor | 
| Trigger & Button 8 | Intake Coral | manual mode overrides coral sensor | 
| Button 2 | Manual elevator | Controls elevator speed manually |
| Button 5 on cluster | Coral station | Moves elevator and tilt to coral station height |
| Button 3 on cluster | L2 position | Moves elevator and tilt to L2 height |
| Button 4 on cluster | L3 position | Moves elevator and tilt to L3 height |
| Button 6 on cluster | L4 position | Moves elevator and tilt to L4 height |
| Button 9 | stow elevator | moves elevator to bottom and zero tilt |
| Button 10 | climber | manually move the climber with joystick |
| Button 11 | manual tilt | control the tilt with the joystick |

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

#### Left Joystick (Operator) - Test Mode
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
