# FTC 2025/2026 - Team 26644
Team documentation / FAQs
## Connect to the robot from your personal device
1. Connect to the robot's WiFi. The SSID should be `26644-RC`, and the password is `Love2code!`.
2. Next, open up the terminal on _Android Studio_. You can do this by pressing `Alt+F12`.
3. In the terminal, type the following command and hit enter.
```
adb connect 192.168.43.1
```
Note: If your terminal says that the command is not found, you can try this command instead:
```
C:\Users\$env:username\AppData\Local\Android\Sdk\platform-tools\adb.exe connect 192.168.43.1
```
## API
Documentation for tele-op and autonomous interfaces

### Classes
- `Robot`
- `OmniDriveTrain`
- `Launcher`

### Robot

The `Robot` class is used as the main entry point for tele-op and autonomous programs. To create a new robot program one would start by creating a new Java class, and extending it with `Robot` like so:
```java
import org.firstinspires.ftc.teamcode.lib.Robot;

public class MyRobotProgram extends Robot {}
```
Your new program will either be `TeleOp` or `Autonomous`. For this example we will create an `Autonomous` program. Add the following to the top of your class:

```java
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Robot")
public class MyRobotProgram extends Robot {}
```
This class requires that we define a method called `setup`. This method is responsible for inititalizing our drive train, launcher, and other peripherals needed. This is the minimum amount of required code: 
```java
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Robot")
public class MyRobotProgram extends Robot {
    OmniDriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new OmniDriveTrain(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }
}
```
Finally, to actually start writing code, our main entry point is in a method called `run`. Define the method like so to start writing routines for the robot.

```java
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Robot")
public class MyRobotProgram extends Robot {
    OmniDriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new OmniDriveTrain(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void run() throws InterruptedException {
      // Autonomous routines go here...
    }
}
```
### OmniDriveTrain
The drive train class is an easy way to interface with the motors that drive the mecanum wheels for the robot. We defined a variable `driveTrain`. We will use this `OmniDriveTrain` object as an example


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
