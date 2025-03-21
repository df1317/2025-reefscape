# Docs

All the fancy quick reference stuff goes here.

### Teleop

#### Xbox Controller (Driver) [`Port 0`]

| Binding         | Action                 | Description                                                                    |
| --------------- | ---------------------- | ------------------------------------------------------------------------------ |
| `Button A`      | Zero gyro              | Resets gyro heading                                                            |
| `Button X`      | Auto Target left Reef  | automatically target and drive to left reef; will rumble if further than 1.5m  |
| `Button B`      | Auto Target right Reef | automatically target and drive to right reef; will rumble if further than 1.5m |
| `Button Y`      | kill auto drive        | restores manual control to the drive system                                    |
| `Left Bumper`   | Lock drive             | Locks drivebase                                                                |
| `Right Trigger` | play music             | Kraken sings the duck song                                                     |
| `Right Bumper`  | toggles robot relative | <----                                                                          |

#### Left Joystick (Operator) [`Port 1`]

| Binding                | Action              | Description                                     |
| ---------------------- | ------------------- | ----------------------------------------------- |
| `Trigger`              | Auto Intake / Eject | based on coral sensor                           |
| `Trigger` & `Button 7` | Eject Coral         | manual mode overrides coral sensor              |
| `Trigger` & `Button 8` | Intake Coral        | manual mode overrides coral sensor              |
| `Button 2`             | Manual elevator     | Controls elevator speed manually                |
| `Button 5` on cluster  | Coral station       | Moves elevator and tilt to coral station height |
| `Button 3` on cluster  | L2 position         | Moves elevator and tilt to L2 height            |
| `Button 4` on cluster  | L3 position         | Moves elevator and tilt to L3 height            |
| `Button 6` on cluster  | L4 position         | Moves elevator and tilt to L4 height            |
| `Button 9`             | Climber descend     | Controls climber down                           |
| `Button 10`            | Climber climb       | Controls climber up                             |
| `Button 11`            | manual tilt         | control the tilt with the joystick              |
| `Button 12`            | zero tilt encoder   | sets the encoder to zero                        |

### Test Mode

#### Xbox Controller (Driver)

| Binding         | Action            | Description                 |
| --------------- | ----------------- | --------------------------- |
| `X Button`      | Fake vision       | Adds fake vision reading    |
| `Back Button`   | Center modules    | Centers swerve modules      |
| `B Button`      | Zero elevator     | Zeros elevator position     |
| `Y Button`      | Drive to distance | Drives forward 1m at 0.2m/s |
| `Left Bumper`   | Play music        | Plays music                 |
| `Right Bumper`  | Play music        | Plays music                 |
| `Right Trigger` | Eject             | Runs eject                  |

#### Left Joystick (Operator) - Test Mode

| Binding     | Action          | Description                       |
| ----------- | --------------- | --------------------------------- |
| `Trigger`   | Manual elevator | Controls elevator speed manually  |
| `Button 7`  | Demo mode       | Cycles through elevator positions |
| `Button 8`  | Climb           | Controls climber up               |
| `Button 9`  | Descend         | Controls climber down             |
| `Button 10` | Zero tilt       | Zeros tilt position               |
| `Button 11` | Tilt down       | Nudges tilt down                  |
| `Button 12` | Tilt up         | Nudges tilt up                    |
| `Button 2`  | Tilt SysID      | Runs tilt system identification   |

#### Right Joystick (Operator) [`Port 2`]

| Binding     | Action            | Description                                   |
| ----------- | ----------------- | --------------------------------------------- |
| `Button 9`  | Elevator SysID    | Runs elevator system identification           |
| `Button 10` | Tilt SysID        | Runs tilt system identification               |
| `Button 11` | Angle motor SysID | Runs swerve angle motor system identification |
| `Button 12` | Drive motor SysID | Runs swerve drive motor system identification |

## CAN IDs

| Component        | Location    | ID   |
| ---------------- | ----------- | ---- |
| Drive Motor      | Front Right | `12` |
| Drive Motor      | Front Left  | `13` |
| Drive Motor      | Back Right  | `14` |
| Drive Motor      | Back Left   | `15` |
|                  |             |
| Turn Motor       | Front Right | `16` |
| Turn Motor       | Front Left  | `17` |
| Turn Motor       | Back Right  | `18` |
| Turn Motor       | Back Left   | `19` |
|                  |             |
| CanCoder         | Front Right | `20` |
| CanCoder         | Front Left  | `21` |
| CanCoder         | Back Right  | `22` |
| CanCoder         | Back Left   | `23` |
|                  |             |
| Score Motor      | 1           | `24` |
| Score Motor      | 2           | `25` |
| Score Tilt Motor | -           | `28` |
|                  |             |
| Elevator Motor   | Left        | `26` |
| Elevator Motor   | Right       | `27` |
|                  |             |
| Climber Motor    | Beefy       | `29` |

## DIO Constants

| Sensor                    | Port |
| ------------------------- | ---- |
| Coral Sensor              | `1`  |
| Elevator Encoder Left     | `2`  |
| Elevator Encoder Right    | `3`  |
| Homing Tilt Clicky Switch | `4`  |
