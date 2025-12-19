# FTC 2025/2026 - Team 26644
Team documentation / FAQs
## Connect to the robot from your personal device
1. Connect to the robot's WiFi. The SSID should be `26644-RC`, and the password is `Love2code!`.
2. Next, open up the terminal on _Android Studio_. You can do this by pressing `Alt+F12`.
3. In the terminal, type the following command and hit enter.
```
adb connect 192.168.43.1
```
Note for Windows users: If your terminal says that the command is not found, you can try this command instead:
```
C:\Users\$env:username\AppData\Local\Android\Sdk\platform-tools\adb.exe connect 192.168.43.1
```
## API
Documentation for tele-op and autonomous interfaces

### Classes
- `Robot`
- `Launcher`
- `DriveTrain`

### Robot

The `Robot` class is used as the main entry point for tele-op and autonomous programs. To create a new robot program, start by creating a new Java class and extending it with `Robot` like so:
```java
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.lib.Robot;

public class MyRobotProgram extends Robot {}
```
#### Autonomous
Your new program will either be `TeleOp` or `Autonomous`. For this example we will create an `Autonomous` program. Add the following to the top of your class:
```java
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MyAutonomous", group="Robot")
public class MyRobotProgram extends Robot {}
```
This class requires that we define a method called `setup`. This method is responsible for initializing our drive train, launcher, and other peripherals needed. ABSOLUTELY NO LOGIC SHOULD GO HERE, THE ROBOT WILL NOT RESPOND TO IT, AND YOU WILL PROBABLY THROW AN EXCEPTION. This is the minimum amount of required code to initialize the entire robot:

```java
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MyAutonomous", group="Robot")
public class MyRobotProgram extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
    }
}
```
Finally, to actually start writing code, our main entry point is in a method called `run`. Define the method like so to start writing routines for the robot.
```java
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MyAutonomous", group="Robot")
public class MyRobotProgram extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void run() throws InterruptedException {
        // Autonomous routines go here...
    }
}
```
Optionally, you can define a `cycle` method to have code run as a cycle in a while loop. No need to
handle the program exiting gracefully.
```java
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MyAutonomous", group="Robot")
public class MyRobotProgram extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void cycle(double delta) throws InterruptedException {
        // These instructions loop until the program ends
        
        // the `delta` variable represents how much time has passed since the last cycle.
        // This can be useful for some algorithms.
    }
}
```
If `run` is defined, cycle WILL NOT run unless the member variable `cycleShouldRun` is set to `true`. If `cycleShouldRun` is set to `true` when `run` is also defined, `cycle` will start cycling after `run`

#### TeleOp
For tele-op, nothing changes except for the annotation at the top of the class. In tele-op mode, all of the same methods are available to you.
```java
package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MyTeleOp", group="Robot")
public class MyRobotProgram extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void cycle(double delta) throws InterruptedException {
        // These instructions loop until the program ends
    }
}
```
The tele-op code for this year is already defined under `TeleOp.java`. If you have changes to make, _please_ make them there. No one should be creating a new TeleOp program in 2025/2026.

### DriveTrain
The drive train class is an easy way to interface with the motors that drive the mecanum wheels for the robot. We defined a variable `driveTrain`. We will use this `DriveTrain` object as an example here. Let's begin our example program by moving the robot 1 foot forward, then 1/2 a foot backward.
```java
@Override
public void run() throws InterruptedException {
    driveTrain.moveForwardFor(12.0);
    driveTrain.moveBackwardFor(6.0);
}
```
We can also strafe left and right. This block of code will strafe the robot 1 foot to the left and back.
```java
@Override
public void run() throws InterruptedException {
    driveTrain.strafeLeftFor(12.0);
    driveTrain.strafeRightFor(12.0);
}
```
You can also rotate the robot. Only the right direction works at the moment. This will turn the robot right for the specified degrees. This block will turn the robot at a right angle
```java
@Override
public void run() throws InterruptedException {
    driveTrain.turnfor(90);
}
```
#### Coaches:
- Parker
- Binod
#### Members:
- Adhav
- Alexander
- Dylan
- Sruti
- Suhruth
- Vidyuth
