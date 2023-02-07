# Arcade Drive Drivetrain Documentation

This document describes a modified version the drivetrain used in the 2022 and 2023 seasons of FRC, and how it is implemented in the code.

## "Arcade" Drive and Inputs

In an Arcade drive system, we take input in the form of two values, both between -1 and 1.

1. Drive: how fast the robot should move forward or backward (blue)
2. Rotation: how fast the robot should rotate left or right (red)

when drive is positive, the robot moves forward, when speed is negative, the robot moves backward. Likewise, when rotation is positive, the robot rotates right (clockwise), and when rotation is negative, the robot rotates left (counterclockwise).

<p align="center">
  <img src="Images\labeledController.png" alt = "example of a labeled controller, with the triggers marked with blue arrows and the left joystick X axis marked with red arrows"/ width="600">
</p>

This is in contrast with a more simplistic "Tank" drive system, wherein 2 inputs in $[-1, 1]$ are also taken, but instead, they represent the outputs of the left and right sides of the robot directly (e.g. the Y axes of the Left and Right joysticks.)

**Note:** since the rotation input is taken from the X position of the left joystick, it is already has a range of [-1, 1], however, since we use the L and R triggers for our drive input, which are in the range [0, 1], we get our drive input with the expression: $\text{Drive} = \text{Forward} - \text{Backward}$, giving us a value with the desired range and behavior.

## Our Desired Output

From these inputs, we want to determine the output of the motors on each side of the drivetrain. That is, given some drive and rotation, we want to find out how fast we want the wheels on the left and right side of the robot to spin.

<p align="center">
  <img src="Images\labeledDrivetrain.png" alt = "example of a labeled drivetrain, pulled from an online cad model"/>
</p>

> `IMPORTANT NOTE`: notice how the motors are mirrored in this image of an example drivetrain; this means when it comes time to actually apply our output values, we would need to invert our values on one side to produce the desired effect.

## Mapping Inputs to Outputs

To find a way to map our input space (drive, rotation) to our output space (left, right), we can start by looking at the extremes, seeing what the logical outputs are, and then trying to work from there to find our intermediate outputs.

Specifically, we should first take a look at the input combinations $(-1, -1), (-1, 0), (-1,1)$, $(0,-1)$, $(0,0)$, $(0,1)$, $(1,-1)$, $(1,0)$, and $(1,1)$.

<p align="center">
  <img src="Images\mapping.png" alt = "3x3 grid of input-output mappings"/>
</p>

By doing so, we explore a few intuitive combinations that should make sense to us. For the cases of $(\text{Drive}, 0)$ like $(1,0)$ and $(-1,0)$, we would expect our robot to drive directly forwards/ backwards, meaning our outputs for both sides should be the same.

Likewise for the cases of $(0, \text{Rotation})$ like $(0,1)$ and $(0,-1)$, we would expect our robot to pivot in place, rotating either CW or CCW without forwards or backwards movement. Here, both sides move in opposite directions, producing a rotation with the center of the robot as the pivot.

For our corner cases, since their rotation inputs aren't $0$, they'd produce a rotation in the same direction as either $(0,1)$ or $(0,-1)$, but they differ in the pivot choice in order to also produce movement in the desired direction.

For example: since the input $(1,1)$ expresses a command to both move forward and rotate clockwise, the only way we can satisfy both these conditions is to run the left wheels, but not the right wheels. This way, we'd rotate the robot counterclockwise, but now pivoting around the right side of the robot (instead of the center), which moves the robot forwards slightly.

## Getting a Formula

From these 9 points, we can start to figure out how we'd get outputs for values which aren't just $0$, $-1$, or $1$, which can take a bit a trial/guesswork. An inital observation we can make is that the 0 points for our left output on the line $y = -x$, (or in context: $\text{Drive}=-\text{Rotation}$), at inputs $(1,-1), (0,0), $ and $(-1,1)$, with values to its upper-right being positive, and values to its lower-left being negative

We can see a similar relationship with the right output and the line $\text{Drive} = \text{Rotation}$ for the right motor output, with the upper-left and lower-right values being positive and negative respectively.

With that, we can produce a set of formulae: $$\begin{cases} \text{left}(\text{Drive},\text{Rotation}) = \text{Drive}+\text{Rotation} \\ \text{right}(\text{Drive},\text{Rotation}) = \text{Drive}-\text{Rotation} \end{cases}$$ which is derived from the two equations above, like for the right output: (as our output is 0 on the line) $$\text{Drive} = \text{Rotation} \implies \text{right}(\text{Drive},\text{Rotation}) = 0 = \text{Drive} - \text{Rotation}$$

Since both $\text{Rotation}-\text{Drive}$ and $\text{Drive}-\text{Rotation}$ present valid solutions, we'd crosscheck with our points above. Since for input $(1,-1)$, the right output is positive, we'd choose the second expression, since $(1)-(-1)$ yields $2$, whereas $(-1)-(1)$ yields $-2$.

At this point you should also notice that both values are outside the range of possible values for our outputs (which is range $[-1,1]$). But here we can simply clamp our values to our desired range using `MathUtil.clamp(val, -1, 1)`, which would produce the following outputs:

<p align="center">
  <img src="Images\simpleImplOutput.png" alt = "motor output graphs"/>
</p>

Looking at our 9 previous known points, we see that these formulas matches the outputs, and in each quadrant, the combination of the two outputs produces the intended directional and rotational movement...on the surface.

## Trouble in Paradise

 But looking closer at our graphs, we see a problem that came as a result of our clamping from earlier. In quadrants II and IV for the left motor, and quadrants I and III for the right motor, we got a nice smooth gradient that encompasses the entirety of the input space, which is desirable.

 However, on the other quadrants, we see an issue where by the halfway points (e.g. $(0.5,0.5)$ for the left motor) the motor outputs maxes out, and we see no change between there and the corner of the graph when we move along the diagonal.

<p align="center">
  <img src="Images\outputIssue.png" alt = "motor output vectors"/>
</p>

## Fixing the issue
  
To remedy this, we'd want to create an artificial gradient of sorts, one that starts off at $0$ when given input $(0,0)$, then increases to $1$ when moving towards $(1,1)$, $(1,0)$, or $(0,1)$, like for example:

<p align="center">
  <img src="Images\\desiredQuadrantI.png" alt = "motor output graphs"/>
</p>

How might such a gradient be made? Looking at the graph, we see the gradient is split by the line $\text{Drive} = \text{Rotation}$.

- Above the line, when $\text{Drive} \gt \text{Rotation}$, the output is just the value of $\text{Drive}$
- Below the line, when $\text{Drive} \gt \text{Rotation}$, the output is just the value of $\text{Rotation}$

So, one way we could do so is by taking the maximum of the $\text{Drive}$ and $\text{Rotation}$ inputs. A similar process can be used for rectifying the formula in Quadrant III, except since we're dealing with negatives there, we'd take the maximum of the absolute values, and then multiply it by the sign of the $\text{Drive}$: $$\text{gradient}(\text{Drive}, \text{Rotation}) = \text{sign}(\text{Drive})\cdot\max(|\text{Drive}|, |\text{Rotation}|)$$

Take a step back here and try to convince yourself that this formula also works in creating a gradient in all 4 quadrants, as we'll be utilizing it for both the right and left outputs.

Applying our new gradients, we have a set of more desirable, but also more complicated formulae: (for brevity, I'll simply refer to the result of `gradient(Drive, Rotation)` for some input as `grad`) $$\begin{cases} \text{left}(\text{Drive},\text{Rotation}) = \begin{cases} \text{QI} = \text{grad}\\ \text{QII}  = \text{Drive}+\text{Rotation} \\ \text{QIII}  = \text{grad} \\ \text{QIV}  = \text{Drive}+\text{Rotation} \end{cases} \\\\  \text{right}(\text{Drive},\text{Rotation}) = \begin{cases} \text{QI} = \text{Drive}-\text{Rotation}\\ \text{QII}  = \text{grad} \\ \text{QIII}  = \text{Drive}-\text{Rotation} \\ \text{QIV}  = \text{grad} \end{cases} \end{cases}$$

Which translates to the following, much nicer looking plot output mapping:

<p align="center">
  <img src="Images\outputs.png" alt = "motor output graphs"/>
</p>

which, in addition, fixes our previous issue with output scaling

<p align="center">
  <img src="Images\outputCorrected.png" alt = "motor output graphs"/>
</p>

## Translating To Code

With our mathematical formulae in hand, we can translate it into code which looks much like the code in `DriveTrain.java`, although perhaps with a bit of difference in naming

```java
public void arcadeDrive(double drive, double rotation){
    // copySign accomplishes the "* sign(drive)" operation
    double grad = Math.copySign(Math.max(
        Math.abs(drive),
        Math.abs(rotation)
    ), drive);
    double leftMotorOutput, rightMotorOutput;

    if (drive >= 0) {
        if (rotation >= 0) {
            // Quadrant 1 (+R, +S)
            leftMotorOutput = grad;
            rightMotorOutput = drive - rotation;
        } else {
            // Quadrant 2 (-R, +S)
            leftMotorOutput = drive + rotation;
            rightMotorOutput = grad;
        }
    } else {
        if (rotation >= 0) {
            // Quadrant 4 (+R, -S)
            leftMotorOutput = drive + rotation;
            rightMotorOutput = grad;
        } else {
            // Quadrant 3 (-R, -S)
            leftMotorOutput = grad;
            rightMotorOutput = drive - rotation;
        }
    }

    // NOTE: right side output is flipped, since motors are mirrored on drivetrain
    leftLeader.set(leftMotorOutput);
    rightLeader.set(-rightMotorOutput);
}
```
