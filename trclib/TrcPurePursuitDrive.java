/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a platform independent Pure Pursuit drive for holonomic or non-holonomic robots.
 * Essentially, a pure pursuit drive navigates the robot to chase a point along the path. The point to chase is
 * chosen by intersecting a proximity circle centered on the robot with a specific radius with the path, and chasing
 * the "furthest" intersection. The smaller the radius is, the more "tightly" the robot will follow a path, but it
 * will be more prone to oscillation and sharp turns. A larger radius will tend to smooth out turns and corners. Note
 * that the error tolerance must be less than the proximity radius, so choose them accordingly.
 * <p>
 * A path consists of an array of waypoints, specifying position, velocity, and heading. All other properties
 * of the TrcWaypoint object may be ignored.The path may be low resolution, as this automatically interpolates between
 * waypoints. If you want the robot to maintain heading, call setMaintainHeading(true) and it will ignore all the
 * heading values. Otherwise, call setMaintainHeading(false), ensure that the heading tolerance and pid coefficients
 * are set, and it will follow the heading values specified by the path. Note that MaintainHeading is only supported
 * for holonomic robots.
 * <p>
 * A somewhat similar idea is here:
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552 or
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 * <p>
 * Note that this paper is for non-holonomic robots. This means that all the turning radius stuff isn't very relevant.
 * Technically, we could impose limits on the turning radius as a function of robot velocity and max rot vel, but that's
 * unnecessarily complicated, in my view. Additionally, it does point injection instead of interpolation, and path
 * smoothing, which we don't do, since a nonzero proximity radius will naturally smooth it anyway.
 */
public class TrcPurePursuitDrive
{
    private static final String moduleName = "TrcPurePursuitDrive";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private static final boolean verbosePidInfo = false;
    private TrcDbgTrace dbgTrace = null;

    public enum InterpolationType
    {
        LINEAR(1), QUADRATIC(2), CUBIC(3), QUARTIC(4), QUADRATIC_INV(2), CUBIC_INV(3), QUARTIC_INV(4);

        private final int value;

        InterpolationType(int value)
        {
            this.value = value;
        }   //InterpolationType

        public int getValue()
        {
            return value;
        }   //getValue

    }   //enum InterpolationType

    private final String instanceName;
    private final TrcDriveBase driveBase;
    private volatile double proximityRadius;    // Volatile so it can be changed at runtime
    private volatile double posTolerance;       // Volatile so it can be changed at runtime
    private final TrcPidController xPosPidCtrl, yPosPidCtrl, turnPidCtrl, velPidCtrl;
    private final TrcWarpSpace warpSpace;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private double moveOutputLimit = Double.POSITIVE_INFINITY;
    private double rotOutputLimit = Double.POSITIVE_INFINITY;
    private InterpolationType interpolationType = InterpolationType.LINEAR;

    private TrcDbgTrace msgTracer = null;
    private TrcRobotBattery battery = null;
    private boolean logRobotPoseEvents = false;
    private boolean tracePidInfo = false;

    private TrcPath path;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;
    private int pathIndex = 1;
    private TrcPose2D referencePose;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param proximityRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param xPosPidCoeff specifies the position PID coefficients for X.
     * @param yPosPidCoeff specifies the position PID coefficients for Y.
     * @param turnPidCoeff specifies the turn PID coefficients.
     * @param velPidCoeff specifies the velocity PID coefficients.
     */
    public TrcPurePursuitDrive(String instanceName, TrcDriveBase driveBase, double proximityRadius,
                               double posTolerance, double turnTolerance,
                               TrcPidController.PidCoefficients xPosPidCoeff,
                               TrcPidController.PidCoefficients yPosPidCoeff,
                               TrcPidController.PidCoefficients turnPidCoeff,
                               TrcPidController.PidCoefficients velPidCoeff)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;

        if (xPosPidCoeff == null || driveBase.supportsHolonomicDrive())
        {
            this.driveBase = driveBase;
        }
        else
        {
            throw new IllegalArgumentException(
                "xPosPidCoeff is provided but drive base does not support holonomic drive!");
        }

        xPosPidCtrl = xPosPidCoeff == null? null:
            new TrcPidController(instanceName + ".xPosPid", xPosPidCoeff, posTolerance, driveBase::getXPosition);
        yPosPidCtrl = new TrcPidController(
            instanceName + ".yPosPid", yPosPidCoeff, posTolerance, driveBase::getYPosition);
        turnPidCtrl = new TrcPidController(
            instanceName + ".turnPid", turnPidCoeff, turnTolerance, driveBase::getHeading);
        // We are not checking velocity being onTarget, so we don't need velocity tolerance.
        velPidCtrl = new TrcPidController(
            instanceName + ".velPid", velPidCoeff, 0.0, this::getVelocityInput);

        setPositionToleranceAndProximityRadius(posTolerance, proximityRadius);

        turnPidCtrl.setAbsoluteSetPoint(true);
        velPidCtrl.setAbsoluteSetPoint(true);
        turnPidCtrl.setNoOscillation(true);

        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        driveTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTask", this::driveTask);
    }   //TrcPurePursuitDrive

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
     * Set both the position tolerance and proximity radius.
     *
     * @param posTolerance    sepcifies the distance at which the controller will stop itself.
     * @param proximityRadius specifies the distance between the robot and next following point.
     */
    public synchronized void setPositionToleranceAndProximityRadius(Double posTolerance, Double proximityRadius)
    {
        if (posTolerance != null && proximityRadius != null && posTolerance >= proximityRadius)
        {
            throw new IllegalArgumentException("Position tolerance must be less than proximityRadius!");
        }

        if (posTolerance != null)
        {
            yPosPidCtrl.setTargetTolerance(posTolerance);
            if (xPosPidCtrl != null)
            {
                xPosPidCtrl.setTargetTolerance(posTolerance);
            }
            this.posTolerance = posTolerance;
        }

        if (proximityRadius != null)
        {
            this.proximityRadius = proximityRadius;
        }
    }   //setPositionToleranceAndProximityRadius

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param posTolerance The distance at which the controller will stop itself.
     */
    public void setPositionTolerance(double posTolerance)
    {
        setPositionToleranceAndProximityRadius(posTolerance, null);
    }   //setPositionTolerance

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param proximityRadius specifies the distance between the robot and next following point.
     */
    public void setProximityRadius(double proximityRadius)
    {
        setPositionToleranceAndProximityRadius(null, proximityRadius);
    }   //setProximityRadius

    /**
     * Set the turn tolerance for the closed loop control on turning. Only applicable if not maintaining heading.
     *
     * @param turnTolerance The turn tolerance, in degrees. Should be positive.
     */
    public synchronized void setTurnTolerance(double turnTolerance)
    {
        turnPidCtrl.setTargetTolerance(turnTolerance);
    }   //setTurnTolerance

    /**
     * Sets the pid coefficients for the X position controller. This will work in the middle of an operation as well.
     *
     * @param xPidCoefficients The new PID coefficients for the X position controller.
     */
    public synchronized void setXPositionPidCoefficients(TrcPidController.PidCoefficients xPidCoefficients)
    {
        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.setPidCoefficients(xPidCoefficients);
        }
    }   //setXPositionPidCoefficients

    /**
     * Sets the pid coefficients for the Y position controller. This will work in the middle of an operation as well.
     *
     * @param yPidCoefficients The new PID coefficients for the Y position controller.
     */
    public synchronized void setYPositionPidCoefficients(TrcPidController.PidCoefficients yPidCoefficients)
    {
        yPosPidCtrl.setPidCoefficients(yPidCoefficients);
    }   //setYPositionPidCoefficients

    /**
     * Sets the pid coefficients for both X and Y position controllers. This will work in the middle of an operation as
     * well.
     *
     * @param pidCoefficients The new PID coefficients for both X and Y position controllers.
     */
    public synchronized void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        setXPositionPidCoefficients(pidCoefficients);
        setYPositionPidCoefficients(pidCoefficients);
    }   //setPositionPidCoefficients

    /**
     * Sets the pid coefficients for the turn controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the heading controller.
     */
    public synchronized void setTurnPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        turnPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setTurnPidCoefficients

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     * Note that velocity controllers should have an F term as well.
     *
     * @param pidCoefficients The new PIDF coefficients for the velocity controller.
     */
    public synchronized void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        velPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setVelocityPidCoefficients

    /**
     * Sets the movement output power limit.
     *
     * @param limit specifies the output power limit for movement (X and Y).
     */
    public synchronized void setMoveOutputLimit(double limit)
    {
        moveOutputLimit = Math.abs(limit);
    }   //setMoveOutputLimit

    /**
     * Sets the rotation output power limit.
     *
     * @param limit specifies the output power limit for rotation.
     */
    public synchronized void setRotOutputLimit(double limit)
    {
        rotOutputLimit = Math.abs(limit);
    }   //setRotOutputLimit

    /**
     * Configure the method of interpolating between waypoints. Methods ending with INV will favor the ending point.
     *
     * @param interpolationType The type of interpolation to use.
     */
    public synchronized void setInterpolationType(InterpolationType interpolationType)
    {
        this.interpolationType = interpolationType == null ? InterpolationType.LINEAR : interpolationType;
    }   //setInterpolationType

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param logRobotPoseEvents specifies true to log robot pose events, false otherwise.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public synchronized void setMsgTracer(
        TrcDbgTrace tracer, boolean logRobotPoseEvents, boolean tracePidInfo, TrcRobotBattery battery)
    {
        this.msgTracer = tracer;
        this.logRobotPoseEvents = logRobotPoseEvents;
        this.tracePidInfo = tracePidInfo;
        this.battery = battery;
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param logRobotPoseEvents specifies true to log robot pose events, false otherwise.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean logRobotPoseEvents, boolean tracePidInfo)
    {
        setMsgTracer(tracer, logRobotPoseEvents, tracePidInfo, null);
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        setMsgTracer(tracer, false, false, null);
    }   //setMsgTracer

    /**
     * This method returns the field position of the target waypoint of the path (i.e. the last waypoint in the path).
     *
     * @return field position of the last waypoint in the path.
     */
    public TrcPose2D getTargetFieldPosition()
    {
        TrcPose2D targetPose = null;

        if (referencePose != null && path != null && path.getSize() > 0)
        {
            targetPose = referencePose.addRelativePose(path.getLastWaypoint().pose);
        }

        return targetPose;
    }   //getTargetFieldPosition

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public synchronized void start(
        TrcPath path, TrcEvent onFinishedEvent, double timeout, Double maxVel, Double maxAccel)
    {
        if (path == null || path.getSize() == 0)
        {
            throw new IllegalArgumentException("Path cannot be null or empty!");
        }

        cancel();

        this.path = path;

        if (maxVel != null && maxAccel != null)
        {
            path.trapezoidVelocity(maxVel, maxAccel);
        }

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;
        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        pathIndex = 1;

        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.reset();
        }
        else
        {
            //
            // For non-holonomic drive base, the robot heading must be pointing to the endpoint of the line segment.
            // So, we must ignore the provided startpoint heading and compute our own based on the startpoint heading
            // and the relative angle of the startpoint from the endpoint.
            //
            for (int i = 0; i < path.getSize() - 1; i++)
            {
                TrcWaypoint startPoint = path.getWaypoint(i);
                TrcWaypoint endPoint = path.getWaypoint(i + 1);
                startPoint.pose.angle = Math.toDegrees(Math.atan2(endPoint.pose.x - startPoint.pose.x,
                                                                  endPoint.pose.y - startPoint.pose.y));
            }
        }
        yPosPidCtrl.reset();
        turnPidCtrl.reset();
        velPidCtrl.reset();

        referencePose = driveBase.getFieldPosition();
        driveTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, TrcEvent onFinishedEvent, Double maxVel, Double maxAccel)
    {
        start(path, onFinishedEvent, 0.0, maxVel, maxAccel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, Double maxVel, Double maxAccel)
    {
        start(path, null, 0.0, maxVel, maxAccel);
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
        start(path, onFinishedEvent, timeout, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     */
    public void start(TrcPath path, TrcEvent onFinishedEvent)
    {
        start(path, onFinishedEvent, 0.0, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     */
    public void start(TrcPath path)
    {
        start(path, null, 0.0, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
            TrcEvent onFinishedEvent, double timeout, TrcPose2D startingPose, boolean incrementalPath,
            Double maxVel, Double maxAccel, TrcPose2D... poses)
    {
        TrcPathBuilder pathBuilder = new TrcPathBuilder(startingPose, incrementalPath);

        for (TrcPose2D pose: poses)
        {
            pathBuilder.append(pose);
        }

        start(pathBuilder.toRelativeStartPath(), onFinishedEvent, timeout, maxVel, maxAccel);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent onFinishedEvent, TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel,
        TrcPose2D... poses)
    {
        start(onFinishedEvent, 0.0, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel, TrcPose2D... poses)
    {
        start(null, 0.0, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent onFinishedEvent, double timeout, TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(onFinishedEvent, timeout, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(TrcEvent onFinishedEvent, TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(onFinishedEvent, 0.0, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, 0.0, startingPose, incrementalPath, null, null, poses);
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

    /**
     * Stops PurePursuit drive.
     */
    private synchronized void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop();
    }   //stop

    /**
     * This method is called by the Velocity PID controller to get the polar magnitude of the robot's velocity.
     *
     * @return robot's velocity magnitude.
     */
    private double getVelocityInput()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getVelocityInput

    /**
     * This task is called periodically to calculate the next target point on the path. The next target point on
     * the path has a distance of followDistance from the current robot position intersecting with the path segment
     * towards the end of the endpoint of the path segment.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = moduleName + ".driveTask";
        TrcPose2D robotPose = driveBase.getPositionRelativeTo(referencePose, false);
        TrcWaypoint targetPoint = getFollowingPoint(robotPose);
        TrcPose2D relativePose = targetPoint.pose.relativeTo(robotPose, true);

        if (xPosPidCtrl != null)
        {
            xPosPidCtrl.setTarget(relativePose.x);
        }
        yPosPidCtrl.setTarget(relativePose.y);
        turnPidCtrl.setTarget(targetPoint.pose.angle, warpSpace);
        velPidCtrl.setTarget(targetPoint.velocity);

        double xPosPower = xPosPidCtrl != null? xPosPidCtrl.getOutput(): 0.0;
        double yPosPower = yPosPidCtrl.getOutput();
        double turnPower = turnPidCtrl.getOutput();
        double velPower = velPidCtrl.getOutput();
        double theta = Math.atan2(relativePose.x, relativePose.y);
        xPosPower = xPosPidCtrl == null? 0.0: TrcUtil.clipRange(xPosPower + velPower * Math.sin(theta),
                                                                -moveOutputLimit, moveOutputLimit);
        yPosPower = TrcUtil.clipRange(yPosPower + velPower * Math.cos(theta), -moveOutputLimit, moveOutputLimit);
        turnPower = TrcUtil.clipRange(turnPower, -rotOutputLimit, rotOutputLimit);

        if (debugEnabled)
        {
//            TrcMotorController[] motors = driveBase.getMotors();
//            StringBuilder msg = new StringBuilder("motors=(");
//            for (TrcMotorController motor: motors)
//            {
//                msg.append(String.format("%s=%.0f ", motor, motor.getPosition()));
//            }
//            msg.append(")");
//            dbgTrace.traceInfo(funcName, msg.toString());
            dbgTrace.traceInfo(
                funcName, "[%d:%.6f] RobotPose=%s,TargetPose=%s,relPose=%s",
                pathIndex, TrcUtil.getModeElapsedTime(), robotPose, targetPoint.pose, relativePose);
            dbgTrace.traceInfo(
                funcName,
                "RobotVel=%.1f,TargetVel=%.1f,xError=%.1f,yError=%.1f,turnError=%.1f,velError=%.1f,theta=%.1f," +
                "xPower=%.1f,yPower=%.1f,turnPower=%.1f,velPower=%.1f",
                getVelocityInput(), targetPoint.velocity, xPosPidCtrl != null? xPosPidCtrl.getError(): 0.0,
                yPosPidCtrl.getError(), turnPidCtrl.getError(), velPidCtrl.getError(), Math.toDegrees(theta),
                xPosPower, yPosPower, turnPower, velPower);
//            // Temp debugging
//            yPosPidCtrl.printPidInfo(dbgTrace, true);
        }

        // If we have timed out or finished, stop the operation.
        boolean timedOut = TrcUtil.getCurrentTime() >= timedOutTime;
        boolean posOnTarget = TrcUtil.magnitude(relativePose.x, relativePose.y) <= posTolerance;
        boolean headingOnTarget = turnPidCtrl.isOnTarget();
        if (timedOut || (pathIndex == path.getSize() - 1 && posOnTarget && headingOnTarget))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.signal();
            }
            stop();
        }
        else if (xPosPidCtrl != null)
        {
            driveBase.holonomicDrive(xPosPower, yPosPower, turnPower);
        }
        else
        {
            driveBase.arcadeDrive(yPosPower, turnPower);
        }

        if (msgTracer != null)
        {
            if (logRobotPoseEvents)
            {
                msgTracer.logEvent(instanceName, "RobotPose", "pose=\"%s\"", driveBase.getFieldPosition());
            }

            if (tracePidInfo)
            {
                if (xPosPidCtrl != null) xPosPidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
                yPosPidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
                turnPidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
                velPidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
            }
        }
    }   //driveTask

    /**
     * Returns a weighted value between given values.
     *
     * @param startValue specifies the start value.
     * @param endValue specifies the end value.
     * @param weight specifies the weight between the values.
     * @return weighted value between the given values.
     */
    private double interpolate(double startValue, double endValue, double weight)
    {
        if (!TrcUtil.inRange(weight, 0.0, 1.0))
        {
            throw new IllegalArgumentException("Weight must be in range [0,1]!");
        }

        switch (interpolationType)
        {
            case LINEAR:
            case QUADRATIC:
            case CUBIC:
            case QUARTIC:
                weight = Math.pow(weight, interpolationType.getValue());
                break;

            case QUADRATIC_INV:
            case CUBIC_INV:
            case QUARTIC_INV:
                weight = Math.pow(weight, 1.0 / interpolationType.getValue());
                break;
        }

        return (1.0 - weight) * startValue + weight * endValue;
    }   //interpolate

    /**
     * Interpolates a waypoint that's weighted between two given waypoints.
     *
     * @param point1 specifies the start point of the path segment.
     * @param point2 specifies the end point of the path segment.
     * @param weight specifies the weight between the two provided points.
     * @param robotPose specifies the robot's position, set to null for holonomic drivebase.
     * @return weighted interpolated waypoint.
     */
    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight, TrcPose2D robotPose)
    {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.pose.x, point2.pose.x, weight);
        double y = interpolate(point1.pose.y, point2.pose.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = interpolate(point1.velocity, point2.velocity, weight);
        double acceleration = interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = interpolate(point1.jerk, point2.jerk, weight);

        double heading;
        double turningRadius = proximityRadius + posTolerance;
        if (robotPose == null || robotPose.distanceTo(point2.pose) <= turningRadius)
        {
            if (robotPose != null)
            {
                //
                // For non-holonomic drivebase and the end-waypoint is within the robot's proximity circle +
                // posTolerance, we will interpolate the heading weight differently than holonomic drivebase.
                // The heading weight is the percentage distance of the robot position to the end-waypoint over
                // proximity radius.
                //
                weight = 1 - point2.pose.distanceTo(point1.pose)*(1 - weight)/turningRadius;
            }
            heading = interpolate(
                point1.pose.angle, warpSpace.getOptimizedTarget(point2.pose.angle, point1.pose.angle), weight);
        }
        else
        {
            //
            // For non-holonomic drivebase, maintain the robot heading pointing to the end-waypoint unless the
            // end-waypoint is within the robot's proximity circle.
            //
            TrcPose2D endpointPose = point2.pose.clone();
            endpointPose.angle = point1.pose.angle;
            heading = point1.pose.angle + endpointPose.relativeTo(robotPose).angle;
        }

        return new TrcWaypoint(timestep, new TrcPose2D(x, y, heading), position, velocity, acceleration, jerk);
    }   //interpolate

    /**
     * This method calculates the waypoint on the path segment that intersects the robot's proximity circle that is
     * closest to the end point of the path segment. The algorithm is based on this article:
     * https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
     *
     * @param startWaypoint specifies the start point of the path segment.
     * @param endWaypoint specifies the end point of the path segment.
     * @param robotPose specifies the robot's position.
     * @return calculated waypoint.
     */
    private TrcWaypoint getFollowingPointOnSegment(
        TrcWaypoint startWaypoint, TrcWaypoint endWaypoint, TrcPose2D robotPose)
    {
        // Find intersection of path segment with the proximity circle of the robot.
        RealVector startVector = startWaypoint.getPositionPose().toPosVector();
        RealVector endVector = endWaypoint.getPositionPose().toPosVector();
        RealVector robotVector = robotPose.toPosVector();

        RealVector startToEnd = endVector.subtract(startVector);
        RealVector robotToStart = startVector.subtract(robotVector);
        // Solve quadratic formula
        double a = startToEnd.dotProduct(startToEnd);
        double b = 2 * robotToStart.dotProduct(startToEnd);
        double c = robotToStart.dotProduct(robotToStart) - proximityRadius * proximityRadius;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            // No valid intersection.
            return null;
        }
        else
        {
            // line is a parametric equation, where t=0 is start waypoint, t=1 is end waypoint of the line segment.
            discriminant = Math.sqrt(discriminant);
            //
            // t1 and t2 represent the relative positions of the intersection points on the line segment. If they are
            // in the range of 0.0 and 1.0, they are on the line segment. Otherwise, the intersection points are
            // outside of the line segment. If the relative position is towards 0.0, it is closer to the start
            // waypoint of the line segment. If the relative position is towards 1.0, it is closer to the end
            // waypoint of the line segment.
            //
            // t represents the furthest intersection point (the one closest to the end waypoint of the line segment).
            //
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            double t = Math.max(t1, t2);

            if (!TrcUtil.inRange(t, 0.0, 1.0))
            {
                //
                // The furthest intersection point is not on the line segment, so skip this segment.
                //
                return null;
            }

            return interpolate(startWaypoint, endWaypoint, t, xPosPidCtrl == null? robotPose: null);
        }
    }   //getFollowingPointOnSegment

    /**
     * Determines the next target point for Pure Pursuit Drive to follow.
     *
     * @param robotPose specifies the robot's location.
     * @return next target point for the robot to follow.
     */
    private TrcWaypoint getFollowingPoint(TrcPose2D robotPose)
    {
        //
        // Find the next segment that intersects with the proximity circle of the robot.
        // If there are tiny segments that are completely within the proximity circle, we will skip them all.
        //
        for (int i = Math.max(pathIndex, 1); i < path.getSize(); i++)
        {
            // If there is a valid intersection, return it.
            TrcWaypoint interpolated = getFollowingPointOnSegment(
                path.getWaypoint(i - 1), path.getWaypoint(i), robotPose);
            if (interpolated != null)
            {
                pathIndex = i;
                return interpolated;
            }
        }
        //
        // Found no intersection. The robot must be off-path. Just proceed to the immediate next waypoint.
        //
        return path.getWaypoint(pathIndex);
    }   //getFollowingPoint

}   //class TrcPurePursuitDrive
