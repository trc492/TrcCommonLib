/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.ArrayList;

/**
 * This class implements a platform independent servo motor. Typically, this class is to be extended by a platform
 * dependent servo class and must provide a set of abstract methods. This makes sure the rest of the TrcLib classes
 * can access the servo without any knowledge of platform dependent implementations.
 */
public abstract class TrcServo
{
    protected static final String moduleName = "TrcServo";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method inverts the servo motor direction.
     *
     * @param inverted specifies the servo direction is inverted if true.
     */
    public abstract void setInverted(boolean inverted);

    /**
     * This method checks if the servo motor direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    public abstract boolean isInverted();

    /**
     * This method sets the logical position of the servo motor.
     *
     * @param position specifies the logical position of the servo motor in the range of [0.0, 1.0].
     */
    public abstract void setLogicalPosition(double position);

    /**
     * This method returns the logical position of the servo. In general, servo motors do not provide real time
     * position feedback unless it has an encoder. Therefore, it will only return real time position if there is
     * an encoder. Otherwise, it will return the position set by the last setLogicalPosition call.
     *
     * @return logical position of the servo motor in the range of [0.0, 1.0].
     */
    public abstract double getLogicalPosition();

    /**
     * This method stops a continuous servo. It doesn't do anything if the servo is not continuous.
     */
    public abstract void stopContinuous();

    public static final double SERVO_CONTINUOUS_FWD_MAX = 1.0;
    public static final double SERVO_CONTINUOUS_REV_MAX = 0.0;
    public static final double SERVO_CONTINUOUS_STOP = 0.5;

    private static final double DEF_PHYSICAL_MIN    = 0.0;
    private static final double DEF_PHYSICAL_MAX    = 1.0;
    private static final double DEF_LOGICAL_MIN     = 0.0;
    private static final double DEF_LOGICAL_MAX     = 1.0;
    protected static TrcElapsedTimer servoSetPosElapsedTimer = null;

    private final ArrayList<TrcServo> followers = new ArrayList<>();
    private final String instanceName;
    private final boolean continuous;
//    private final TrcDigitalInput lowerLimitSwitch;
//    private final TrcDigitalInput upperLimitSwitch;
    private final TrcTimer timer;
    private final TrcTaskMgr.TaskObject servoTaskObj;
    private boolean taskEnabled = false;
//    private boolean calibrating = false;

    private double physicalMin = DEF_PHYSICAL_MIN;
    private double physicalMax = DEF_PHYSICAL_MAX;
    private double logicalMin = DEF_LOGICAL_MIN;
    private double logicalMax = DEF_LOGICAL_MAX;

    private double targetPosition = 0.0;
    private double currStepRate = 0.0;
    private double prevTime = 0.0;
    private double currPosition = 0.0;
    private double currPower = 0.0;
    private double maxStepRate = 0.0;
    private double minPos = 0.0;
    private double maxPos = 1.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the servo.
     * @param continuous specifies true if it is a continuous servo, false otherwise.
     */
    public TrcServo(String instanceName, boolean continuous)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.continuous = continuous;
//        this.lowerLimitSwitch = lowerLimitSwitch;
//        this.upperLimitSwitch = upperLimitSwitch;
        timer = new TrcTimer(instanceName);

        servoTaskObj = TrcTaskMgr.createTask(instanceName + ".servoTask", this::servoTask);
        TrcTaskMgr.createTask(instanceName + ".stopTask", this::stopTask).registerTask(TrcTaskMgr.TaskType.STOP_TASK);
    }   //TrcServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the servo.
     */
    public TrcServo(String instanceName)
    {
        this(instanceName, false);
    }   //TrcServo

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
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (servoSetPosElapsedTimer == null)
            {
                servoSetPosElapsedTimer = new TrcElapsedTimer("TrcServo.setPos", 2.0);
            }
        }
        else
        {
            servoSetPosElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to use for printing elapsed time info.
     */
    public static void printElapsedTime(TrcDbgTrace tracer)
    {
        if (servoSetPosElapsedTimer != null)
        {
            servoSetPosElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

    /**
     * This method adds a following servo to the followers list.
     *
     * @param followingServo specifies the following servo.
     * @return true if the servo is added successfully, false if it is already in the list.
     */
    public boolean addFollower(TrcServo followingServo)
    {
        boolean success = false;

        if (!followers.contains(followingServo))
        {
            success = followers.add(followingServo);
        }

        return success;
    }   //addFollower

    /**
     * This method removes the following servo from the followers list.
     *
     * @param followingServo specifies the servo to be removed from the list.
     * @return true if the servo is successfully removed from the list, false if the servo was not in the list.
     */
    public boolean removeFollower(TrcServo followingServo)
    {
        return followers.remove(followingServo);
    }   //removeFollower

    /**
     * This method stops the servo.
     */
    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (continuous)
        {
            stopContinuous();
        }

        if (taskEnabled)
        {
            setTaskEnabled(false);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    /**
     * This method checks if the servo is a continuous servo.
     *
     * @return true if the servo is continuous, false otherwise.
     */
    public boolean isContinuous()
    {
        return continuous;
    }   //isContinuous

    /**
     * This method returns the physical position value of the servo motor. Generally, servo motors do not provide
     * real time position feedback unless the platform dependent implementation supports sensors such as encoders.
     * Otherwise, it will only return the position set by the last setPosition call.
     *
     * @return physical position of the servo, could be in degrees if setPhysicalRange is called to set the range in
     *         degrees.
     */
    public double getPosition()
    {
        return toPhysicalPosition(getLogicalPosition());
    }   //getPosition

    /**
     * This method sets the servo motor position. By default, the servo maps its physical position the same as its
     * logical position [0.0, 1.0]. However, if setPhysicalRange was called, it could map a real world physical
     * range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event or notifier is given,
     * it sets event or call the notifier after the given amount of time has expired.
     * <p>
     * Servo motor operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree. On a continuous servo, 0.0 is rotating full speed in reverse, 0.5 is to stop the motor and 1.0 is
     * rotating the motor full speed forward. Again, motor direction can be inverted if setInverted is called.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     * @param event specifies an event object to signal when the timeout event has expired.
     * @param notifier specifies the notifier to call when the timeout event has expired.
     */
    public void setPosition(double position, double timeout, TrcEvent event, TrcNotifier.Receiver notifier)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "pos=%f,event=%s,notifier=%s", position, event, notifier);
        }

        if (!continuous)
        {
            double logicalPos = toLogicalPosition(TrcUtil.clipRange(position, physicalMin, physicalMax));
            setLogicalPosition(logicalPos);

            for (TrcServo servo : followers)
            {
                servo.setLogicalPosition(logicalPos);
            }

            if (timeout > 0.0 && (event != null || notifier != null))
            {
                timer.set(timeout, event, notifier);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPosition

    /**
     * This method sets the servo motor position. By default, the servo maps its physical position the same as its
     * logical position [0.0, 1.0]. However, if setPhysicalRange was called, it could map a real world physical
     * range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event or notifier is given,
     * it sets event or call the notifier after the given amount of time has expired.
     * <p>
     * Servo motor operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree. On a continuous servo, 0.0 is rotating full speed in reverse, 0.5 is to stop the motor and 1.0 is
     * rotating the motor full speed forward. Again, motor direction can be inverted if setInverted is called.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     * @param event specifies an event object to signal when the timeout event has expired.
     */
    public void setPosition(double position, double timeout, TrcEvent event)
    {
        setPosition(position, timeout, event, null);
    }   //setPosition

    /**
     * This method sets the servo motor position. By default, the servo maps its physical position the same as its
     * logical position [0.0, 1.0]. However, if setPhysicalRange was called, it could map a real world physical
     * range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event or notifier is given,
     * it sets event or call the notifier after the given amount of time has expired.
     * <p>
     * Servo motor operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree. On a continuous servo, 0.0 is rotating full speed in reverse, 0.5 is to stop the motor and 1.0 is
     * rotating the motor full speed forward. Again, motor direction can be inverted if setInverted is called.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     * @param timeout specifies a maximum time value the operation should be completed in seconds.
     * @param notifier specifies the notifier to call when the timeout event has expired.
     */
    public void setPosition(double position, double timeout, TrcNotifier.Receiver notifier)
    {
        setPosition(position, timeout, null, notifier);
    }   //setPosition

    /**
     * This method sets the servo motor position. By default, the servo maps its physical position the same as its
     * logical position [0.0, 1.0]. However, if setPhysicalRange was called, it could map a real world physical
     * range (e.g. [0.0, 180.0] degrees) to the logical range of [0.0, 1.0]. If an event or notifier is given,
     * it sets event or call the notifier after the given amount of time has expired.
     * <p>
     * Servo motor operates on logical position. On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree.
     * For a 90-degree servo, 0->0deg, 1->90deg. If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is
     * at 0-degree. On a continuous servo, 0.0 is rotating full speed in reverse, 0.5 is to stop the motor and 1.0 is
     * rotating the motor full speed forward. Again, motor direction can be inverted if setInverted is called.
     *
     * @param position specifies the physical position of the servo motor. This value may be in degrees if
     *                 setPhysicalRange is called with the degree range.
     */
    public void setPosition(double position)
    {
        setPosition(position, 0.0, null, null);
    }   //setPosition

    /**
     * This method sets the servo to the specifies position but with the specified step rate in effect controlling
     * the speed to get there.
     *
     * @param position specifies the target position.
     * @param stepRate specifies the stepping rate to get there (physicalPos/sec).
     */
    public synchronized void setPosition(double position, double stepRate)
    {
        final String funcName = "setPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f,stepRate=%f", position, stepRate);
        }

        if (!continuous)
        {
            this.targetPosition = position;
            this.currStepRate = Math.abs(stepRate);
            this.prevTime = TrcUtil.getCurrentTime();
            this.currPosition = getPosition();
            setTaskEnabled(true);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPosition

    /**
     * This method sets the stepping mode characteristics.
     *
     * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
     * @param minPos specifies the minimum position.
     * @param maxPos specifies the maximum position.
     */
    public synchronized void setStepMode(double maxStepRate, double minPos, double maxPos)
    {
        final String funcName = "setStepMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "maxStepRate=%f,minPos=%f,maxPos=%f", maxStepRate, minPos, maxPos);
        }

        if (!continuous)
        {
            this.maxStepRate = maxStepRate;
            this.minPos = minPos;
            this.maxPos = maxPos;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setStepMode

    /**
     * This method sets the power just like a regular motor but for a servo. If it is a continuous servo, it will
     * set it running with different speed. If it is a regular servo, it will change its step rate.
     *
     * @param power specifies how fast the servo will turn.
     */
    public void setPower(double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%f", power);
        }

        power = TrcUtil.clipRange(power);
        if (continuous)
        {
//            if (lowerLimitSwitch != null && lowerLimitSwitch.isActive() && power < 0.0 ||
//                upperLimitSwitch != null && upperLimitSwitch.isActive() && power > 0.0)
//            {
//                //
//                // One of the limit switches is hit, so stop!
//                //
//                stopContinuous();
//            }
//            else
//            {
                power = TrcUtil.scaleRange(
                    power, -1.0, 1.0, SERVO_CONTINUOUS_REV_MAX, SERVO_CONTINUOUS_FWD_MAX);
                setLogicalPosition(power);
//            }
        }
        else if (!isTaskEnabled())
        {
            //
            // Not in stepping mode, so start stepping mode.
            // If we are calibrating, cancel calibration.
            //
//            calibrating = false;
            setPosition(power > 0.0? maxPos: minPos, Math.abs(power)*maxStepRate);
        }
        else if (power != 0.0)
        {
            //
            // We are already in stepping mode, just change the stepping parameters.
            //
            targetPosition = power > 0.0? maxPos: minPos;
            currStepRate = Math.abs(power)*maxStepRate;
        }
        else
        {
            //
            // We are stopping.
            //
            stop();
        }
        currPower = power;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method returns the last set power value.
     *
     * @return last power set to the motor.
     */
    public double getPower()
    {
        final String funcName = "getPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", currPower);
        }

        return currPower;
    }   //getPower

    /**
     * This method sets the physical range of the servo motor. This is typically
     * used to set a 180-degree servo to have a range of 0.0 to 180.0 instead of
     * the logical range of 0.0 to 1.0.
     *
     * @param physicalMin specifies the minimum value of the physical range.
     * @param physicalMax specifies the maximum value of the physical range.
     */
    public void setPhysicalRange(double physicalMin, double physicalMax)
    {
        final String funcName = "setPhysicalRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "phyMin=%f,phyMax=%f", physicalMin, physicalMax);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (physicalMin >= physicalMax)
        {
            throw new IllegalArgumentException("max must be greater than min.");
        }

        this.physicalMin = physicalMin;
        this.physicalMax = physicalMax;
    }   //setPhysicalRange

    /**
     * This method sets the logical range of the servo motor. This is typically used to limit the logical range
     * of the servo to less than the 0.0 to 1.0 range. For example, one may limit the logical range to 0.2 to 0.8.
     *
     * @param logicalMin specifies the minimum value of the logical range.
     * @param logicalMax specifies the maximum value of the logical range.
     */
    public void setLogicalRange(double logicalMin, double logicalMax)
    {
        final String funcName = "setLogicalRange";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "logicalMin=%f,logicalMax=%f", logicalMin, logicalMax);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (logicalMin >= logicalMax)
        {
            throw new IllegalArgumentException("max must be greater than min.");
        }

        this.logicalMin = logicalMin;
        this.logicalMax = logicalMax;
    }   //setLogicalRange

    /**
     * This method is called to convert a physical position to a logical position. It will make sure the physical
     * position is within the physical range and scale it to the logical range. Note: this method is only callable
     * by classes extending this class.
     *
     * @param physicalPosition specifies the physical position to be converted
     * @return converted logical position.
     */
    public double toLogicalPosition(double physicalPosition)
    {
        final String funcName = "toLogicalPosition";
        physicalPosition = TrcUtil.clipRange(physicalPosition, physicalMin, physicalMax);
        double logicalPosition = TrcUtil.scaleRange(physicalPosition, physicalMin, physicalMax, logicalMin, logicalMax);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "phyPos=%f", physicalPosition);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", logicalPosition);
        }

        return logicalPosition;
    }   //toLogicalPosition

    /**
     * This method is called to convert a logical position to a physical position.
     * It will make sure the logical position is within the logical range and scale
     * it to the physical range.
     * Note: this method is only callable by classes extending this class.
     *
     * @param logicalPosition specifies the logical position to be converted.
     * @return converted physical position.
     */
    public double toPhysicalPosition(double logicalPosition)
    {
        final String funcName = "toPhysicalPosition";

        logicalPosition = TrcUtil.clipRange(logicalPosition, logicalMin, logicalMax);
        double physicalPosition = TrcUtil.scaleRange(logicalPosition, logicalMin, logicalMax, physicalMin, physicalMax);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "logPos=%f", logicalPosition);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", physicalPosition);
        }

        return physicalPosition;
    }   //toPhysicalPosition

    /**
     * This method enables/disables the enhanced servo task for performing step rate speed control or zero
     * calibration.
     *
     * @param enabled specifies true to enable task, false to disable.
     */
    private synchronized void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "enabled=%b", enabled);
        }

        if (enabled)
        {
            servoTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
        }
        else
        {
            servoTaskObj.unregisterTask();
//            calibrating = false;
        }
        taskEnabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setTaskEnabled

    /**
     * This method checks if the task is enabled.
     *
     * @return true if task is enabled, false otherwise.
     */
    private synchronized boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s", taskEnabled);
        }

        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method is called periodically to check whether the servo has reached target. If not, it will calculate
     * the next position to set the servo to according to its step rate.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void servoTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "servoTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
//            if (calibrating)
//            {
//                if (logicalRangeLow == -1.0 && lowerLimitSwitch.isActive())
//                {
//                    //
//                    // We finished calibrating the low range, now start calibrating the high range.
//                    //
//                    logicalRangeLow = servo1.toLogicalPosition(currPosition);
//                    setPosition(physicalRangeMax, currStepRate);
//                }
//                else if (logicalRangeHigh == -1.0 && upperLimitSwitch.isActive())
//                {
//                    //
//                    // We finished calibrating the high range, we are done calibrating.
//                    // Note: stop() will set calibrating to false.
//                    //
//                    logicalRangeHigh = servo1.toLogicalPosition(currPosition);
//                    servo1.setLogicalRange(logicalRangeLow, logicalRangeHigh);
//                    if (servo2 != null)
//                    {
//                        servo2.setLogicalRange(logicalRangeLow, logicalRangeHigh);
//                    }
//                    stop();
//                }
//                else if (currPosition == targetPosition)
//                {
//                    //
//                    // Somehow, we reached the end and did not trigger a limit switch. Let's abort.
//                    // Note: stop() will set calibrating to false.
//                    //
//                    stop();
//                }
//            }

            double currTime = TrcUtil.getCurrentTime();
            double deltaPos = currStepRate * (currTime - prevTime);

            if (currPosition < targetPosition)
            {
                currPosition += deltaPos;
                if (currPosition > targetPosition)
                {
                    currPosition = targetPosition;
                }
            }
            else if (currPosition > targetPosition)
            {
                currPosition -= deltaPos;
                if (currPosition < targetPosition)
                {
                    currPosition = targetPosition;
                }
            }
            else //if (!calibrating)
            {
                //
                // We have reached target and we are not calibrating, so we are done.
                //
                stop();
            }
            prevTime = currTime;

            setPosition(currPosition);
//            if (servo1 != null)
//            {
//                servo1.setPosition(currPosition);
//            }
//
//            if (servo2 != null)
//            {
//                servo2.setPosition(currPosition);
//            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //servoTask

    /**
     * This method is called when the competition mode is about to end so it will stop the servo if necessary.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private void stopTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }
        //
        // Note: stop() will set calibrating to false.
        //
        stop();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

}   //class TrcServo
