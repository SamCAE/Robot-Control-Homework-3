# ReadMe

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_rviz.launch
```

```bash
roscd turtlebot3_gazebo; cd scripts; python dwa_simulation.py
```

### Part 1)

```python
def robot_model(V,w,T):
    dt=0.2 #time interval
    #Create a variable to store x,y points and yaw angles
    xy=[[pose[0],pose[1]]]
    thetas=[pose[2]]
    for i in range(int(T/dt)): # Update x,y,theta for a specific interval
        # Fill here !!! 
        xy = xy + [[xy[-1][0] + m.cos(thetas[-1] + w*dt/2)*V*dt, xy[-1][1] + m.sin(thetas[-1] + w*dt/2)*V*dt]]
        thetas = thetas + [thetas[-1] + w*dt]
        pass
    # return with trajectory xy points in [[x1,y1],[x2,y2],...,[xn,yn]] form and last angle of the trajectory
    return xy[1:],thetas[-1]
```

A differential drive robot imposes what is called non-holonomic constraints on establishing its position. For example, the robot cannot move laterally along its axle. A similar nonholonomic constraint is a car that can only turn its front wheels. It cannot move directly sidewise, as parallel parking a car requires a more complicated set of steering manoeuvres. So we cannot simply specify an arbitrary robot pose $(x, y, θ)$ and find the velocities that will get us there.

For the special cases of $v_l = v_r = v$  (robot movng in a straight line) the motion equations become:

$$
\begin{bmatrix}
x^‎\prime\\
y^‎\prime\\
\theta^‎\prime
\end{bmatrix}=\begin{bmatrix}
x + v cos(θ)δt\\
y + v sin(θ)δt\\
θ
\end{bmatrix}
$$

If $v_r = −v_l = v$, then the robot rotates in place and the equations become:

$$
\begin{bmatrix}
x^‎\prime\\
y^‎\prime\\
\theta^‎\prime
\end{bmatrix}=\begin{bmatrix}
x \\
y \\
θ + 2vδt/l
\end{bmatrix}
$$

This motivates a strategy of moving the robot in a straight line, then rotating for a turn in place, and then moving straight again as a navigation strategy for differential drive robots.

### Part 2)

![1.gif](ReadMe%202881ee17d87c4333b957dda17978c405/1.gif)

```bash
Traceback (most recent call last):
  File "dwa_simulation.py", line 117, in <module>
    xydist=distance_to_obstacles(x, y)
  File "dwa_simulation.py", line 73, in distance_to_obstacles
    return min(np.hypot(obstacles[:,0]-x,obstacles[:,1]-y)) # Minimum distance between a point(x,y) and obstacles
ValueError: operands could not be broadcast together with shapes (94,) (95,)
```

The cause of the error could be the fact that the solver was not able to find a possible route that gets the robot to its destination. However, it will be seen in “Part 3” that by changing the objective function constants, this error would be fixed.

### Part 3)

In “Part 2” the objective function constants were:

```python
k_vel,k_angle,k_safety=0.4,0.3,0.3
```

And in order to get a clear difference in the results, the constants were changed one at a time with a noticeable change in values.

**Trial 1:**

```python
k_vel,k_angle,k_safety=1,0.3,0.3
```

![**As the k_vel was increased the mobile robot was able to do sharper turns and reach the destination in less time. Moreover, finding routes became more robust since no errors occurred no matter how complicated the goal was.**](ReadMe%202881ee17d87c4333b957dda17978c405/2.gif)

**As the k_vel was increased the mobile robot was able to do sharper turns and reach the destination in less time. Moreover, finding routes became more robust since no errors occurred no matter how complicated the goal was.**

**Trial 2:**

```python
k_vel,k_angle,k_safety=0.4,1,0.3
```

![**Similar to Trial 1, except that the final position of the robot is slightly off the desired goal position.**](ReadMe%202881ee17d87c4333b957dda17978c405/3.gif)

**Similar to Trial 1, except that the final position of the robot is slightly off the desired goal position.**

**Trial 3:**

```python
k_vel,k_angle,k_safety=0.4,0.3,1
```

![**As k_safety was increased,  the more obstacles were detected, the less adventurous the robot gets and hence it would not even move towards the goal.**](ReadMe%202881ee17d87c4333b957dda17978c405/4.gif)

**As k_safety was increased,  the more obstacles were detected, the less adventurous the robot gets and hence it would not even move towards the goal.**