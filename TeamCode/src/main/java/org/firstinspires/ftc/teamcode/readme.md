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
You should see the robot appear as the device in android studio. Build and upload your code by clicking the play button.

## Quickstart Guide
Get started writing code for the robot by following the guide below. For a more detailed list of
methods, [go to the API documentation](#api).

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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MyAutonomous", group="Robot")
```
This abstract class requires that we define a method called `setup`. This method is responsible for initializing our drive train, launcher, and other peripherals needed. ABSOLUTELY NO LOGIC SHOULD GO HERE, THE ROBOT WILL NOT RESPOND TO IT, AND YOU WILL PROBABLY THROW AN EXCEPTION. This is the minimum amount of required code to initialize the entire robot:

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
@Override
public void cycle(double delta) throws InterruptedException {
    // These instructions loop until the program ends
    
    // the `delta` variable represents how much time has passed since the last cycle.
    // This can be useful for some algorithms.
}
```
#### Important note about how `run` and `cycle` interact
If `run` is defined, cycle WILL NOT run unless the member variable `cycleShouldRun` is set to `true`. If `cycleShouldRun` is set to `true` when `run` is also defined, `cycle` will start cycling after `run`

#### TeleOp
For tele-op, nothing changes except for the annotation at the top of the class. In tele-op mode, all of the same methods are available to you.
```java
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MyTeleOp", group="Robot")
```
The tele-op code for this year is already defined under `TeleOp.java`. If you have changes to make, _please_ make them there. No one should be creating a new TeleOp program in 2025/2026.

### DriveTrain
The drive train class is an easy way to interface with the motors that drive the mecanum wheels for the robot. We defined a variable `driveTrain`. We will use this `DriveTrain` object as an example here. Let's begin our example program by moving the robot 1 foot forward, then 1/2 a foot backward.
```java
@Override
public void run() throws InterruptedException {
    driveTrain.forwardFor(12.0);
    driveTrain.reverseFor(6.0);
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
You can also rotate the robot. This will turn the robot right for the specified degrees. This block will turn the robot at a right angle
```java
@Override
public void run() throws InterruptedException {
    driveTrain.rotateLeftFor(90.0);
    driveTrain.rotateRightFor(180.0);
}
```
### Launcher
The launcher is controlled by a `Launcher` object. Like the `driveTrain` variable, we will use the already defined `launcher` as an example here.
```java
@Override
public void run() throws InterruptedException {
    launcher.spinFlywheel();

    sleep(5000); // wait for 5000 ms before executing the next instruction 

    launcher.feed();

    sleep(5000);

    launcher.stop();
}
```
In this example we start off by starting up the flywheel. The flywheel takes a little bit of time to get up to full speed, so we can instruct the robot to wait for a specified number of milliseconds. For this we are waiting 5 seconds before turning on the feeder motors. Once the feeder is feeding, we wait an additional 5 seconds to allow the robot time to launch the ball(s). After that we need to kill the motors with the `stop` method.

## API
### General
`sleep(long milliseconds)`
- Causes the CPU to do nothing for the specified amount of time. Motors that are engaged will stay engaged.
```java
sleep(2000); // Stop the CPU for 2 seconds
```
### DriveTrain API
`setPower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)`
- Directly control the power of the motors all at once. The actual power output depends on the public member variable `maxPower`.

`setSpeed(double speed)`
- Set the speed of the motors for autonomous motion. Accepts a `double` argument between `0.0` and `1.0` for the speed.
```java
driveTrain.setSpeed(0.5); // Set the robot to half speed
```
`forward()`
- Start the drive motors in the forward direction. 

`forwardFor(double inches)`
- Drive the robot forward for the specified amount of inches. Accepts a `double` argument for the number of inches.
```java
driveTrain.forwardFor(12.0); // Drive the robot forward for 1 foot.
```
`reverse()`
- Start the drive motors in the reverse direction. 

`reverseFor(double inches)`
- Drive the robot in the reverse direction for the specified amount of inches. Accepts a `double` argument for the number of inches.
```java
driveTrain.reverseFor(12.5); // Drive the robot backward for 1 foot and 1/2 inch.
```

`strafeLeft()`
- Start the motors to make the robot strafe to the left. 

`strafeLeftFor(double inches)`
- Make the robot strafe to the left for the specified amount of inches. Accepts a `double` argument for the number of inches.
```java
driveTrain.strafeLeftFor(6.5); // Have the robot strafe left for 6 1/2 inches.
```

`strafeRight()`
- Start the motors to make the robot strafe to the right. 

`strafeRightFor(double inches)`
- Make the robot strafe to the right for the specified amount of inches. Accepts a `double` argument for the number of inches.
```java
driveTrain.strafeRightFor(3); // Have the robot strafe right for 3 inches.
```

`rotateLeft()`
- Start rotating the robot to the left. 

`rotateLeftFor(double angle)`
- Start rotating to the left for the specified number of degrees. Accepts a `double` argument for the number of degrees.
```java
driveTrain.rotateLeftFor(90); // Turn the robot at a left right angle
```

`rotateRight()`
- Start rotating the robot to the right. 

`rotateRightFor(double angle)`
- Start rotating to the right for the specified number of degrees. Accepts a `double` argument for the number of degrees.
```java
driveTrain.rotateRightFor(90); // Turn the robot at a right right angle
```
`stop()`
- Stop all drive train motion.

### Launcher API
`launchFor(double seconds)`
- Start spinning the flywheel, and feed the balls into the launcher as soon as the flywheel is up to speed. The flywheel will attempt to launch for as long as the specified time.
```java
launcher.launchFor(5.5); // Start flywheel and launch for 5 1/2 seconds
```

`feed()`
- Start feeding balls into the flywheel area.

`feedIfReady()`
- Start feeding balls into the flywheel area only if the flywheel is up to speed.

`backFeed()`
- Reverse feed balls out from the flywheel area.

`stopFeeder()`
- Stop the feeder motor.

`isSpinning()`
- Returns `true` if the flywheel is currently spinning under power.

`spinFlywheel()`
- Start the flywheel motor.

`stopFlywheel()`
- Stop the flywheel motor.

`toggleFlywheel()`
- If the flywheel is off it will turn on and vice versa.

`stop()`
- Stops the flywheel and the feeder motors.


