/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.trclib;

import org.apache.commons.math3.linear.RealVector;

/**
 * This class implements a platform independent Pure Pursuit drive for holonomic robots.
 * Essentially, a pure pursuit drive navigates the robot to chase a point along the path. The point to chase
 * is chosen by intersecting a circle centered on the robot with a specific radius with the path, and chasing the
 * "furthest" intersection. The smaller the radius is, the more "tightly" the robot will follow a path, but it will be
 * more prone to oscillation and sharp turns. A larger radius will tend to smooth out turns and corners. Note that the
 * error tolerance must be less than the following distance, so choose them accordingly.
 * <p>
 * A path consists of an array of waypoints, specifying position, velocity, and optionally heading. All other properties
 * of the TrcWaypoint object may be ignored.The path may be low resolution, as this automatically interpolates between
 * waypoints. If you want the robot to maintain heading, call setMaintainHeading(true) and it will ignore all the
 * heading values. Otherwise, call setMaintainHeading(false), ensure that the heading tolerance and pid coefficients
 * are set, and it will follow the heading values specified by the path.
 * <p>
 * A somewhat similar idea is here:
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552 or
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 * <p>
 * Note that this paper is for non-holonomic robots. This means that all the turning radius stuff isn't very relevant.
 * Technically, we could impose limits on the turning radius as a function of robot velocity and max rot vel, but that's
 * unnecessarily complicated, in my view. Additionally, it does point injection instead of interpolation, and path
 * smoothing, which we don't do, since a nonzero following distance will naturally smooth it anyway.
 */
public class TrcHolonomicPurePursuitDriveV2
{
    private static final boolean debugEnabled = true;

    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private final TrcPidController turnPidCtrl, velPidCtrl;
    private volatile double velTolerance; // TODO: create setter
    private volatile double posTolerance; // Volatile so it can be changed at runtime
    private volatile double followingDistance; // Volatile so it can be changed at runtime
    private TrcPath path;
    private int pathIndex = 1;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;
    private TrcWarpSpace warpSpace;
    private double startHeading;
    private TrcPose2D referencePose;
    private double moveOutputLimit = Double.POSITIVE_INFINITY;
    private double rotOutputLimit = Double.POSITIVE_INFINITY;
    private double accelFF; // acceleration feedforward

    public TrcHolonomicPurePursuitDriveV2(String instanceName, TrcDriveBase driveBase, double followingDistance,
        double posTolerance, double velTolerance, TrcPidController.PidCoefficients velPidCoeff, double accelFF)
    {
        this(instanceName, driveBase, followingDistance, posTolerance, 180, velTolerance,
            new TrcPidController.PidCoefficients(0), velPidCoeff, accelFF);
    }   //TrcHolonomicPurePursuitDrive

    public TrcHolonomicPurePursuitDriveV2(String instanceName, TrcDriveBase driveBase, double followingDistance,
        double posTolerance, double turnTolerance, double velTolerance, TrcPidController.PidCoefficients turnPidCoeff,
        TrcPidController.PidCoefficients velPidCoeff, double accelFF)
    {
        if (driveBase.supportsHolonomicDrive())
        {
            this.driveBase = driveBase;
        }
        else
        {
            throw new IllegalArgumentException(
                "Only holonomic drive bases supported for this pure pursuit implementation!");
        }

        this.instanceName = instanceName;
        this.accelFF = accelFF;
        this.velTolerance = velTolerance;
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        setPositionToleranceAndFollowingDistance(posTolerance, followingDistance);

        this.turnPidCtrl = new TrcPidController(instanceName + ".turnPid", turnPidCoeff, turnTolerance,
            driveBase::getHeading);
        this.velPidCtrl = new TrcPidController(instanceName + ".velPid", velPidCoeff, velTolerance,
            this::getVelocityInput);

        turnPidCtrl.setAbsoluteSetPoint(true);
        velPidCtrl.setAbsoluteSetPoint(true);

        turnPidCtrl.setNoOscillation(true);

        this.driveTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTask", this::driveTask);
    }   //TrcHolonomicPurePursuitDrive

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * Set the turn tolerance for the closed loop control on turning. Only applicable if not maintaining heading.
     *
     * @param turnTolerance The turn tolerance, in degrees. Should be positive.
     */
    public void setTurnTolerance(double turnTolerance)
    {
        turnPidCtrl.setTargetTolerance(turnTolerance);
    }   //setTurnTolerance

    /**
     * Set the velocity tolerance. The velocity must be less than this value for the controller to finish.
     *
     * @param velTolerance Positive velocity.
     */
    public void setVelTolerance(double velTolerance)
    {
        this.velTolerance = velTolerance;
    }

    /**
     * Set both the position tolerance and following distance.
     *
     * @param posTolerance      The distance at which the controller will stop itself.
     * @param followingDistance The distance between the robot and following point.
     */
    public void setPositionToleranceAndFollowingDistance(double posTolerance, double followingDistance)
    {
        if (posTolerance >= followingDistance)
        {
            throw new IllegalArgumentException("Position tolerance must be less than followingDistance!");
        }

        this.followingDistance = followingDistance;
        this.posTolerance = posTolerance;
    }   //setPositionToleranceAndFollowingDistance

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param posTolerance The distance at which the controller will stop itself.
     */
    public void setPositionTolerance(double posTolerance)
    {
        setPositionToleranceAndFollowingDistance(posTolerance, followingDistance);
    }   //setPositionTolerance

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param followingDistance The distance between the robot and following point.
     */
    public void setFollowingDistance(double followingDistance)
    {
        setPositionToleranceAndFollowingDistance(posTolerance, followingDistance);
    }   //setFollowingDistance

    /**
     * Sets the pid coefficients for the turn controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the heading controller.
     */
    public void setTurnPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        turnPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setTurnPidCoefficients

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     * Note that velocity controllers should have an F term as well.
     *
     * @param pidCoefficients The new PIDF coefficients for the velocity controller.
     */
    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        velPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setVelocityPidCoefficients

    public void setMoveOutputLimit(double limit)
    {
        moveOutputLimit = Math.abs(limit);
    }   //setMoveOutputLimit

    public void setRotOutputLimit(double limit)
    {
        rotOutputLimit = Math.abs(limit);
    }   //setRotOutputLimit

    /**
     * Start following the supplied path using a pure pursuit controller.
     *
     * @param path The path to follow. Must start at (0,0). Velocity is per second.
     */
    public synchronized void start(TrcPath path)
    {
        start(path, null, 0.0);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public synchronized void start(TrcPath path, TrcEvent onFinishedEvent, double timeout)
    {
        if (path == null || path.getSize() == 0)
        {
            throw new IllegalArgumentException("Path cannot be null or empty!");
        }

        cancel();

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;

        this.path = path;
        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        pathIndex = 1;
        startHeading = driveBase.getHeading();

        turnPidCtrl.reset();
        velPidCtrl.reset();

        referencePose = driveBase.getFieldPosition();
        driveTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
    }   //start

    /**
     * Checks if the robot is currently following a path.
     *
     * @return True if the pure pursuit controller is active, false otherwise.
     */
    public synchronized boolean isActive()
    {
        return driveTaskObj.isRegistered();
    }   //isActive

    /**
     * If the controller is currently active, cancel the path following operation. Otherwise, do nothing.
     * If there is an event to signal, mark it as cancelled.
     */
    public synchronized void cancel()
    {
        if (isActive())
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            stop();
        }
    }   //cancel

    private double getVelocityInput()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getVelocityInput

    private synchronized void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop();
    }   //stop

    private synchronized void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        TrcPose2D pose = driveBase.getPositionRelativeTo(referencePose, false);
        TrcWaypoint followingPoint = getFollowingPoint(pose);
        TrcWaypoint targetPoint = getTargetPointDistParameterized(pose);

        double targetVel = targetPoint.velocity;
        velPidCtrl.setTarget(targetVel);
        // Only follow heading if we're not maintaining heading
        turnPidCtrl.setTarget(warpSpace.getOptimizedTarget(targetPoint.heading, pose.angle));

        double turnPower = turnPidCtrl.getOutput();
        double velPower = velPidCtrl.getOutput();
        turnPower = TrcUtil.clipRange(turnPower, -rotOutputLimit, rotOutputLimit);

        double r = velPower + accelFF * targetPoint.acceleration;
        r = TrcUtil.clipRange(r, 0, moveOutputLimit);
        double theta = Math.toDegrees(Math.atan2(followingPoint.x - pose.x, followingPoint.y - pose.y));

        double velocity = TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());

        if (debugEnabled)
        {
            //            TrcDbgTrace.getGlobalTracer().traceInfo("TrcHolonomicPurePursuitDriveV2.driveTask",
            //                "[%.3f] pos=%s, followingPoint=%s, targetPoint=%s, vel=%.2f, targetVel=%.2f, targetAccel=%.2f, pathIndex=%d, r,theta=(%.2f,%.2f)",
            //                TrcUtil.getModeElapsedTime(), pose, followingPoint.getPositionPose(), targetPoint.getPositionPose(),
            //                velocity, targetVel, targetPoint.acceleration, pathIndex, r, theta);
            System.out.printf("%.3f, %.2f, %.2f\n", TrcUtil.getModeElapsedTime(), velocity, targetVel);
        }

        // If we have timed out or finished, stop the operation.

        boolean timedOut = TrcUtil.getCurrentTime() >= timedOutTime;
        boolean posOnTarget = pose.distanceTo(path.getLastWaypoint().getPositionPose()) <= posTolerance;
        boolean headingOnTarget = turnPidCtrl.isOnTarget();
        boolean velOnTarget = velocity <= velTolerance;
        if (timedOut || (posOnTarget && headingOnTarget && velOnTarget))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.signal();
            }
            stop();
        }
        else
        {
            driveBase.holonomicDrive_Polar(r, theta, turnPower, pose.angle - startHeading);
        }
    }   //driveTask

    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight)
    {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.x, point2.x, weight);
        double y = interpolate(point1.y, point2.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = Math.sqrt(interpolate(Math.pow(point1.velocity, 2), Math.pow(point2.velocity, 2), weight));
        double acceleration = point1.acceleration;//interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = point1.jerk;//interpolate(point1.jerk, point2.jerk, weight);
        double heading = interpolate(point1.heading, warpSpace.getOptimizedTarget(point2.heading, point1.heading),
            weight);
        return new TrcWaypoint(timestep, x, y, position, velocity, acceleration, jerk, heading);
    }   //interpolate

    private double interpolate(double start, double end, double weight)
    {
        if (!TrcUtil.inRange(weight, 0.0, 1.0))
        {
            throw new IllegalArgumentException("Weight must be in range [0,1]!");
        }
        return (1.0 - weight) * start + weight * end;
    }   //interpolate

    private TrcWaypoint getFollowingPointOnSegment(TrcWaypoint prev, TrcWaypoint point, TrcPose2D robotPose)
    {
        // Find intersection of path segment with circle with radius followingDistance and center at robot
        RealVector start = prev.getPositionPose().toPosVector();
        RealVector end = point.getPositionPose().toPosVector();
        RealVector robot = robotPose.toPosVector();

        RealVector startToEnd = end.subtract(start);
        RealVector robotToStart = start.subtract(robot);
        // Solve quadratic formula
        double a = startToEnd.dotProduct(startToEnd);
        double b = 2 * robotToStart.dotProduct(startToEnd);
        double c = robotToStart.dotProduct(robotToStart) - followingDistance * followingDistance;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            // No valid intersection.
            return null;
        }
        else
        {
            // line is a parametric equation, where t=0 is start, t=1 is end.
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            double t = Math.max(t1, t2); // We want the furthest intersection
            // If the intersection is not on the line segment, it's invalid.
            if (!TrcUtil.inRange(t, 0.0, 1.0))
            {
                return null;
            }
            return interpolate(prev, point, t);
        }
    }   //interpolatePoints

    TrcWaypoint getTargetPointDistParameterized(TrcPose2D robotPose)
    {
        RealVector robotPos = robotPose.toPosVector();
        double closestDist = Double.MAX_VALUE;
        TrcWaypoint closestPoint = null;
        for (int i = 0; i < path.getSize() - 1; i++)
        {
            RealVector start = path.getWaypoint(i).getPositionPose().toPosVector();
            RealVector startToEnd = path.getWaypoint(i + 1).getPositionPose().toPosVector().subtract(start);
            RealVector startToRobot = robotPos.subtract(start);
            double t = startToRobot.dotProduct(startToEnd) / Math.pow(startToEnd.getNorm(), 2);
            t = TrcUtil.clipRange(t, 0, 1);
            TrcWaypoint point = interpolate(path.getWaypoint(i), path.getWaypoint(i + 1), t);
            double dist = robotPos.getDistance(point.getPositionPose().toPosVector());
            if (dist < closestDist)
            {
                closestDist = dist;
                closestPoint = point;
            }
        }
        return closestPoint;
    }

    TrcWaypoint getFollowingPoint(TrcPose2D robotPose)
    {
        TrcWaypoint last = path.getLastWaypoint();
        if (last.getPositionPose().distanceTo(robotPose) < followingDistance)
        {
            pathIndex = path.getSize() - 1;
            return last;
        }

        for (int i = Math.max(pathIndex, 1); i < path.getSize(); i++)
        {
            // If there is a valid intersection, return it.
            TrcWaypoint interpolated = getFollowingPointOnSegment(path.getWaypoint(i - 1), path.getWaypoint(i),
                robotPose);
            if (interpolated != null)
            {
                pathIndex = i;
                return interpolated;
            }
        }

        // There are no points where the distance to any point is followingDistance.
        // Choose the one closest to followingDistance.
        TrcWaypoint closestPoint = path.getWaypoint(pathIndex);
        for (int i = pathIndex; i < path.getSize(); i++)
        {
            TrcWaypoint point = path.getWaypoint(i);
            if (Math.abs(closestPoint.getPositionPose().distanceTo(robotPose) - followingDistance) >= Math
                .abs(point.getPositionPose().distanceTo(robotPose) - followingDistance))
            {
                closestPoint = point;
                pathIndex = i;
            }
        }
        return closestPoint;
    }   //getFollowingPoint
}   //class TrcPurePursuitDrive

