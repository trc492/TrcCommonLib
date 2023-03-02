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

/**
 * This class implements a platform independent PID controlled motor. A PID controlled motor may consist of one or
 * two physical motors, a position sensor, typically an encoder (or could be a potentiometer). Optionally, it supports
 * a lower limit switch for zero calibration. In addition, it has stall protection support which will detect
 * motor stall condition and will cut power to the motor preventing it from burning out.
 */
public class TrcPidMotor implements TrcExclusiveSubsystem
{
    protected static final String moduleName = "TrcPidMotor";
    protected static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    protected static final boolean debugEnabled = false;

    private static final boolean verbosePidInfo = false;
    private TrcDbgTrace msgTracer = null;
    private TrcRobotBattery battery = null;
    private boolean tracePidInfo = false;

    private static final double MIN_MOTOR_POWER = -1.0;
    private static final double MAX_MOTOR_POWER = 1.0;

    private static final double DEF_BEEP_LOW_FREQUENCY = 440.0;     //in Hz
    private static final double DEF_BEEP_HIGH_FREQUECY = 880.0;     //in Hz
    private static final double DEF_BEEP_DURATION = 0.2;            //in seconds

    /**
     * This class encapsulate all the parameters for a PID motor operation.
     */
    private class TaskParams
    {
        // Zero calibration.
        volatile boolean calibrating = false;
        boolean motor1ZeroCalDone = false;
        boolean motor2ZeroCalDone = false;
        double calPower = 0.0;
        // Stall detection.
        double stallMinPower = 0.0;
        double stallTolerance = 0.0;
        double stallTimeout = 0.0;
        double resetTimeout = 0.0;
        boolean stalled = false;
        double prevPos = 0.0;
        double prevTime = 0.0;
        // PID control.
        double prevTarget = 0.0;
        double currTarget = 0.0;
        boolean holdTarget = false;
        double outputLimit = 1.0;
        TrcEvent notifyEvent = null;
        double timeout = 0.0;
        double currPower = 0.0;
    }   //class TaskParams

    /**
     * This class encapsulates all the parameters required to acquire and release exclusive ownership for the
     * operation.
     */
    private class OwnershipParams
    {
        String owner;
        TrcEvent completionEvent;
        TrcEvent releaseOwnershipEvent;

        OwnershipParams(String owner, TrcEvent completionEvent, TrcEvent releaseOwnershipEvent)
        {
            this.owner = owner;
            this.completionEvent = completionEvent;
            this.releaseOwnershipEvent = releaseOwnershipEvent;
        }   //OwnershipParams
    }   // class OwnershipParams

    protected final String instanceName;
    private final TrcMotor motor1;
    private final TrcMotor motor2;
    private final double syncGain;
    private final TrcPidController.PidParameters pidParams;
    private final TrcDigitalInput lowerLimitSwitch;
    private final double defCalPower;
    private final TrcPidController pidCtrl;
    private final TrcTimer timer;
    private final TrcTaskMgr.TaskObject pidMotorTaskObj;
    private final TrcTaskMgr.TaskObject stopMotorTaskObj;
    private double positionScale = 1.0;
    private double positionOffset = 0.0;
    // Beep device.
    private TrcTone beepDevice = null;
    private double beepLowFrequency = DEF_BEEP_LOW_FREQUENCY;
    private double beepHighFrequency = DEF_BEEP_HIGH_FREQUECY;
    private double beepDuration = DEF_BEEP_DURATION;

    private TaskParams taskParams = new TaskParams();

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies motor1 object.
     * @param motor2 specifies motor2 object. If there is only one motor, this can be set to null.
     * @param syncGain specifies the gain constant for synchronizing motor1 and motor2.
     * @param pidParams specifies the PID parameters for the PID controller.
     * @param useMotorCloseLoopControl specifies true to use motor built-in close loop control, false to use software
     *        PID control.
     * @param lowerLimitSwitch specifies lower limit switch object, null if none.
     * @param defCalPower specifies the default motor power for the calibration.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor1, TrcMotor motor2, double syncGain,
        TrcPidController.PidParameters pidParams, boolean useMotorCloseLoopControl, TrcDigitalInput lowerLimitSwitch,
        double defCalPower)
    {
        if (motor1 == null && motor2 == null)
        {
            throw new IllegalArgumentException("Must have at least one motor.");
        }

        if (pidParams == null)
        {
            throw new IllegalArgumentException("Must provide PID parameters.");
        }

        this.instanceName = instanceName;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.syncGain = syncGain;
        this.pidParams = pidParams;
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.defCalPower = defCalPower;

        if (useMotorCloseLoopControl)
        {
            // Caller does not provide PID params, we will use the motor built-in close loop control.
            pidCtrl = null;
            if (motor1.supportCloseLoopControl())
            {
                motor1.setPidCoefficients(pidParams.pidCoeff);
            }
            else
            {
                throw new IllegalArgumentException("Must provide PID Params if motor does not support close loop control.");
            }
        }
        else
        {
            pidParams.setPidInput(this::getPosition);
            pidCtrl = new TrcPidController(instanceName + ".pidCtrl", pidParams);
        }

        timer = new TrcTimer(instanceName);
        pidMotorTaskObj = TrcTaskMgr.createTask(instanceName + ".pidMotorTask", this::pidMotorTask);
        stopMotorTaskObj = TrcTaskMgr.createTask(instanceName + ".stopMotorTask", this::stopMotorTask);
}   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies motor1 object.
     * @param motor2 specifies motor2 object. If there is only one motor, this can be set to null.
     * @param pidParams specifies the PID parameters for the PID controller.
     * @param lowerLimitSwitch specifies lower limit switch object, null if none.
     * @param defCalPower specifies the default motor power for the calibration.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor1, TrcMotor motor2, TrcPidController.PidParameters pidParams,
        TrcDigitalInput lowerLimitSwitch, double defCalPower)
    {
        this(instanceName, motor1, motor2, 0.0, pidParams, false, lowerLimitSwitch, defCalPower);
    }   //TrcPidMotor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies motor object.
     * @param pidParams specifies the PID parameters for the PID controller.
     * @param useMotorCloseLoopControl specifies true to use motor built-in close loop control, false to use software
     *        PID control.
     * @param lowerLimitSwitch specifies lower limit switch object, null if none.
     * @param defCalPower specifies the default motor power for the calibration.
     */
    public TrcPidMotor(
        String instanceName, TrcMotor motor, TrcPidController.PidParameters pidParams,
        boolean useMotorCloseLoopControl, TrcDigitalInput lowerLimitSwitch, double defCalPower)
    {
        this(instanceName, motor, null, 0.0, pidParams, useMotorCloseLoopControl, lowerLimitSwitch, defCalPower);
    }   //TrcPidMotor

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
     * This method is called to release the exclusive ownership of the subsystem when a certain operation has
     * completed.
     *
     * @param context specifies the releaseOwnership parameters.
     */
    private void releaseOwnership(Object context)
    {
        OwnershipParams ownershipParams = (OwnershipParams) context;

        releaseExclusiveAccess(ownershipParams.owner);
        if (ownershipParams.completionEvent != null)
        {
            if (ownershipParams.releaseOwnershipEvent.isSignaled())
            {
                // setPosition was completed successfully, indicate so in the completion event.
                ownershipParams.completionEvent.signal();
            }
            else
            {
                // setPosition was canceled, indicate so in the completion event.
                ownershipParams.completionEvent.cancel();
            }
        }
    }   //releaseOwnership

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo, TrcRobotBattery battery)
    {
        this.msgTracer = tracer;
        this.tracePidInfo = tracePidInfo;
        this.battery = battery;
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo)
    {
        setMsgTracer(tracer, tracePidInfo, null);
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        setMsgTracer(tracer, false, null);
    }   //setMsgTracer

    /**
     * This method sets the beep device and the beep tones so that it can play beeps when motor stalled.
     *
     * @param beepDevice specifies the beep device object.
     * @param beepLowFrequency specifies the low frequency beep.
     * @param beepHighFrequency specifies the high frequency beep.
     * @param beepDuration specifies the beep duration.
     */
    public void setBeep(TrcTone beepDevice, double beepLowFrequency, double beepHighFrequency, double beepDuration)
    {
        this.beepDevice = beepDevice;
        this.beepLowFrequency = beepLowFrequency;
        this.beepHighFrequency = beepHighFrequency;
        this.beepDuration = beepDuration;
    }   //setBeep

    /**
     * This method sets the beep device so that it can play beeps at default frequencies and duration when motor
     * stalled.
     *
     * @param beepDevice specifies the beep device object.
     */
    public void setBeep(TrcTone beepDevice)
    {
        setBeep(beepDevice, DEF_BEEP_LOW_FREQUENCY, DEF_BEEP_HIGH_FREQUECY, DEF_BEEP_DURATION);
    }   //setBeep

    /**
     * This method sets stall protection. When stall protection is turned ON, it will monitor the motor movement for
     * stalled condition. A motor is considered stalled if:
     * - the power applied to the motor is above or equal to stallMinPower.
     * - the motor has not moved or has moved only within stallTolerance for at least stallTimeout.
     *
     * Note: By definition, holding target position is stalling. If you decide to enable stall protection while
     *       holding target, please make sure to set a stallMinPower much greater the power necessary to hold
     *       position against gravity, for example.
     *
     * @param stallMinPower specifies the minimum motor power to detect stalled condition. If the motor power is
     *                      below stallMinPower, it won't consider it as a stalled condition even if the motor does
     *                      not move.
     * @param stallTolerance specifies the movement tolerance within which is still considered stalled.
     * @param stallTimeout specifies the time in seconds that the motor must stopped before it is declared stalled.
     * @param resetTimeout specifies the time in seconds the motor must be set to zero power after it is declared
     *                     stalled will the stalled condition be reset. If this is set to zero, the stalled condition
     *                     won't be cleared.
     */
    public void setStallProtection(
        double stallMinPower, double stallTolerance, double stallTimeout, double resetTimeout)
    {
        synchronized (taskParams)
        {
            taskParams.stallMinPower = Math.abs(stallMinPower);
            taskParams.stallTolerance = Math.abs(stallTolerance);
            taskParams.stallTimeout = Math.abs(stallTimeout);
            taskParams.resetTimeout = Math.abs(resetTimeout);
        }
    }   //setStallProtection

    /**
     * This method returns the specified motor object.
     *
     * @param primary specifies true to get the primary motor object, false to get the secondary.
     * @return specified motor object.
     */
    public TrcMotor getMotor(boolean primary)
    {
        return primary? motor1: motor2;
    }   //getMotor

    /**
     * This method returns the primary motor object.
     *
     * @return primary motor object.
     */
    public TrcMotor getMotor()
    {
        return getMotor(true);
    }   //getMotor

    /**
     * This method returns the software PID controller. It returns null if using motor close loop control.
     *
     * @return PID controller.
     */
    public TrcPidController getPidController()
    {
        return pidCtrl;
    }   //getPidController

    /**
     * This method returns the state of the PID motor.
     *
     * @return true if PID motor has a PID operation or zero calibration in progress, false otherwise.
     */
    public boolean isPidActive()
    {
        return pidMotorTaskObj.isRegistered();
    }   //isPidActive

    /**
     * This method stops the PID motor. Stopping a PID motor consists of two things: canceling PID and stopping
     * the physical motor(s).
     *
     * @param stopMotor specifies true if also stopping the physical motor(s), false otherwise.
     */
    private void stop(boolean stopMotor)
    {
        final String funcName = "stop";

        // Canceling previous PID operation if any.
        setTaskEnabled(false);
        if (pidCtrl != null)
        {
            pidCtrl.reset();
        }

        if (stopMotor)
        {
            setMotorPower(0.0);
        }

        synchronized (taskParams)
        {
            taskParams.currPower = 0.0;
            taskParams.calibrating = false;
            taskParams.stalled = false;
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "%s.%s: Stopping motor (stop=%s).", moduleName, instanceName, stopMotor);
        }
    }   //stop

    /**
     * This method cancels a previous active PID motor operation.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void cancel(String owner)
    {
        final String funcName = "cancel";

        if (validateOwnership(owner))
        {
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.%s: canceling (owner=%s, event=%s).",
                    moduleName, instanceName, owner, taskParams.notifyEvent);
            }

            synchronized (taskParams)
            {
                timer.cancel();
                // Stop the physical motor(s).
                stop(true);
                taskParams.notifyEvent.cancel();
                taskParams.notifyEvent = null;
            }
        }
    }   //cancel

    /**
     * This method cancels a previous active PID motor operation.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method sets the position scale. Instead of setting PID target with units such as encoder count, one could
     * set the scale to convert the unit to something meaningful such as inches or degrees.
     *
     * @param positionScale specifies the position scale value.
     * @param positionOffset specifies the optional offset that adds to the final position value.
     */
    public void setPositionScaleAndOffset(double positionScale, double positionOffset)
    {
        this.positionScale = positionScale;
        this.positionOffset = positionOffset;
    }   //setPositionScaleAndOffset

    /**
     * This method sets the position scale. Instead of setting PID target with units such as encoder count, one could
     * set the scale to convert the unit to something meaningful such as inches or degrees.
     *
     * @param positionScale specifies the position scale value.
     */
    public void setPositionScale(double positionScale)
    {
        setPositionScaleAndOffset(positionScale, 0.0);
    }   //setPositionScale

    /**
     * This method returns the current scaled motor position.
     *
     * @return scaled motor position.
     */
    public double getPosition()
    {
        int n = 1;
        double pos = motor1.getPosition();

        if (motor2 != null && syncGain != 0.0)
        {
            pos += motor2.getPosition();
            n++;
        }
        pos *= positionScale/n;
        pos += positionOffset;

        return pos;
    }   //getPosition

    /**
     * This method returns the current PID target.
     *
     * @return current motor power.
     */
    public double getTarget()
    {
        return taskParams.currTarget;
    }   //getTarget

    /**
     * This method performs the setPosition operation by enabling the PID motor task.
     *
     * @param params specifies the parameters for the operation.
     */
    private void performSetPosition(TaskParams params)
    {
        final String funcName = "performSetPosition";

        // If a timeout is provided, set the expired time.
        if (params.timeout > 0.0)
        {
            params.timeout += TrcTimer.getCurrentTime();
        }
        // Set a new PID target.
        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: target=%.1f, hold=%s, limit=%.1f, event=%s, timeout=%.3f",
                moduleName, instanceName, params.currTarget, params.holdTarget, params.outputLimit, params.notifyEvent,
                params.timeout);
        }

        if (pidCtrl != null)
        {
            // Use our software PID control.
            // pidMotorTask calls performSetPower which will take care of the output limits.
            pidCtrl.setTarget(params.currTarget);
        }
        else
        {
            // Use motor built-in close loop control.
            motor1.setCloseLoopOutputLimits(-params.outputLimit, params.outputLimit);
            motor1.setMotorPosition((params.currTarget - positionOffset)/positionScale);
        }

        setTaskEnabled(true);
    }   //performSetPosition

    /**
     * This method is called when the setPosition delay timer has expired. It will perform the setPosition.
     *
     * @param context specifies the set position parameters.
     */
    private void setPositionDelayExpired(Object context)
    {
        performSetPosition((TaskParams) context);
    }   //setPositionDelayExpired

    /**
     * This method starts a PID operation by setting the PID target. Generally, when PID operation has reached target,
     * event will be notified and PID operation will end. However, if holdTarget is true, PID operation cannot end
     * because it needs to keep monitoring the position and maintaining it. In this case, it will just notify the event
     * and continue on. The caller is responsible for stopping the PID operation by calling cancel() when done with
     * holding position.
     * If this method specifies an owner and the subsystem was not owned by it, it will acquire exclusive ownership
     * on its behalf and release ownership after the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param completionEvent specifies an event object to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPosition(
        String owner, double delay, double pos, boolean holdTarget, double powerLimit, TrcEvent completionEvent,
        double timeout)
    {
        final String funcName = "setPosition";

        // If a notification event is provided, clear it.
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        // Caller specifies an owner but has not acquired ownership, let's acquire ownership on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            TrcEvent releaseOwnershipEvent = new TrcEvent(instanceName + ".releaseOwnership");
            OwnershipParams ownershipParams = new OwnershipParams(owner, completionEvent, releaseOwnershipEvent);
            releaseOwnershipEvent.setCallback(this::releaseOwnership, ownershipParams);
            completionEvent = releaseOwnershipEvent;
        }

        if (validateOwnership(owner))
        {
            synchronized (taskParams)
            {
                if (isPidActive())
                {
                    // A previous PID operation in progress, stop it but don't stop the motor to prevent jerkiness.
                    stop(false);
                }

                taskParams.currTarget = pos;
                taskParams.holdTarget = holdTarget;
                taskParams.outputLimit = Math.abs(powerLimit);
                taskParams.notifyEvent = completionEvent;
                taskParams.timeout = Math.abs(timeout);
                //
                // Set the PID motor task enabled.
                //
                if (delay > 0.0)
                {
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.%s: delay=%.3f", moduleName, instanceName, delay);
                    }
                    timer.set(delay, this::setPositionDelayExpired, taskParams);
                }
                else
                {
                    performSetPosition(taskParams);
                }
            }
        }
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target. Generally, when PID operation has reached target,
     * event will be notified and PID operation will end. However, if holdTarget is true, PID operation cannot end
     * because it needs to keep monitoring the position and maintaining it. In this case, it will just notify the event
     * and continue on. The caller is responsible for stopping the PID operation by calling cancel() when done with
     * holding position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPosition(
        double delay, double pos, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        setPosition(null, delay, pos, holdTarget, powerLimit, event, timeout);
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target. Generally, when PID operation has reached target,
     * event will be notified and PID operation will end. However, if holdTarget is true, PID operation cannot end
     * because it needs to keep monitoring the position and maintaining it. In this case, it will just notify the event
     * and continue on. The caller is responsible for stopping the PID operation by calling cancel() when done with
     * holding position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPosition(
        String owner, double pos, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        setPosition(owner, 0.0, pos, holdTarget, powerLimit, event, timeout);
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target. Generally, when PID operation has reached target,
     * event will be notified and PID operation will end. However, if holdTarget is true, PID operation cannot end
     * because it needs to keep monitoring the position and maintaining it. In this case, it will just notify the event
     * and continue on. The caller is responsible for stopping the PID operation by calling cancel() when done with
     * holding position.
     *
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies an event object to signal when done.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPosition(double pos, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        setPosition(null, 0.0, pos, holdTarget, powerLimit, event, timeout);
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target. Generally, when PID operation has reached target,
     * event will be notified and PID operation will end. However, if holdTarget is true, PID operation cannot end
     * because it needs to keep monitoring the position and maintaining it. In this case, it will just notify the event
     * and continue on. The caller is responsible for stopping the PID operation by calling cancel() when done with
     * holding position.
     *
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies an event object to signal when done.
     */
    public void setPosition(double pos, boolean holdTarget, double powerLimit, TrcEvent event)
    {
        setPosition(null, 0.0, pos, holdTarget, powerLimit, event, 0.0);
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target.
     *
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     */
    public void setPosition(double pos, boolean holdTarget, double powerLimit)
    {
        setPosition(null, 0.0, pos, holdTarget, powerLimit, null, 0.0);
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target.
     *
     * @param pos specifies the PID target.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     */
    public void setPosition(double pos, boolean holdTarget)
    {
        setPosition(null, 0.0, pos, holdTarget, 1.0, null, 0.0);
    }   //setPosition

    /**
     * This method starts a PID operation by setting the PID target.
     *
     * @param pos specifies the PID target.
     */
    public void setPosition(double pos)
    {
        setPosition(null, 0.0, pos, true, 1.0, null, 0.0);
    }   //setPosition

    /**
     * This class encapsulates the parameters for the setPower call.
     */
    private static class SetPowerParams
    {
        double power;
        double rangeLow;
        double rangeHigh;
        double duration;
        TrcEvent event;
        boolean stopPid;

        SetPowerParams(
            double power, double rangeLow, double rangeHigh, double duration, TrcEvent event, boolean stopPid)
        {
            this.power = power;
            this.rangeLow = rangeLow;
            this.rangeHigh = rangeHigh;
            this.duration = duration;
            this.event = event;
            this.stopPid = stopPid;
        }   //SetPowerParams
    }   //class SetPowerParams

    /**
     * This method returns the current motor power.
     *
     * @return current motor power.
     */
    public double getPower()
    {
        return taskParams.currPower;
    }   //getPower

    /**
     * This method sets the PID motor power. It will also check for stalled condition and cut motor power if stalled
     * detected. It will also check to reset the stalled condition if reset timeout was specified.
     * Note: this method does not handle zero calibration, so do not call this method in zero calibration mode.
     *
     * @param params specifies the SetPowerParams object.
     */
    private void performSetPower(SetPowerParams params)
    {
        if (isPidActive() && params.stopPid)
        {
            // A previous PID operation is still in progress, cancel it. Don't stop the motor to prevent jerkiness.
            stop(false);
        }

        taskParams.stalled = isMotorStalled(params.power);
        if (!resetStall(params.power))
        {
            // Motor was not stalled. Do the normal processing.
            if (pidParams.powerCompensation != null)
            {
                params.power += pidParams.powerCompensation.getCompensation(params.power);
            }
            params.power = TrcUtil.clipRange(params.power, params.rangeLow, params.rangeHigh);
            setMotorPower(params.power);
            taskParams.currPower = params.power;
            if (params.duration > 0.0)
            {
                taskParams.notifyEvent = params.event;
                timer.set(params.duration, this::setPowerDurationExpired);
            }
        }
    }   //performSetPower

    /**
     * This method is called when the setPower delay timer has expired. It will perform the setPower.
     *
     * @param context specifies the set power parameters.
     */
    private void setPowerDelayExpired(Object context)
    {
        performSetPower((SetPowerParams) context);
    }   //setPowerDelayExpired

    /**
     * This method is called when the setPower duration has expired so it can turn off the motor and signal for
     * completion if necessary.
     *
     * @param context specifies the callback context (not used).
     */
    private void setPowerDurationExpired(Object context)
    {
        setMotorPower(0.0);
        if (taskParams.notifyEvent != null)
        {
            taskParams.notifyEvent.signal();
            taskParams.notifyEvent = null;
        }
    }   //setPowerDurationExpired

    /**
     * This method sets the PID motor power. It will also check for stalled condition and cut motor power if stalled
     * detected. It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param delay specifies the delay in seconds before performing the setPower.
     * @param power specifies the motor power.
     * @param rangeLow specifies the power range low limit.
     * @param rangeHigh specifies the power range high limit.
     * @param duration specifies the duration in seconds to run the motor after which it is turned off automatically.
     * @param event specifies the event to signal when duration expired.
     * @param stopPid specifies true to stop previous PID operation, false otherwise.
     */
    public void setPower(
        String owner, double delay, double power, double rangeLow, double rangeHigh, double duration, TrcEvent event,
        boolean stopPid)
    {
        if (validateOwnership(owner))
        {
            SetPowerParams params = new SetPowerParams(power, rangeLow, rangeHigh, duration, event, stopPid);
            if (delay > 0.0)
            {
                timer.set(delay, this::setPowerDelayExpired, params);
            }
            else
            {
                performSetPower(params);
            }
        }
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the motor power.
     * @param rangeLow specifies the range low limit.
     * @param rangeHigh specifies the range high limit.
     */
    public void setPower(String owner, double power, double rangeLow, double rangeHigh)
    {
        setPower(owner, 0.0, power, rangeLow, rangeHigh, 0.0, null, true);
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param power specifies the motor power.
     * @param rangeLow specifies the range low limit.
     * @param rangeHigh specifies the range high limit.
     */
    public void setPower(double power, double rangeLow, double rangeHigh)
    {
        setPower(null, 0.0, power, rangeLow, rangeHigh, 0.0, null, true);
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the motor power.
     * @param duration specifies the duration in seconds to run the motor after which it is turned off automatically.
     * @param event specifies the event to signal when duration expired.
     */
    public void setPower(String owner, double power, double duration, TrcEvent event)
    {
        setPower(owner, 0.0, power, MIN_MOTOR_POWER, MAX_MOTOR_POWER, duration, event, true);
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the motor power.
     */
    public void setPower(String owner, double power)
    {
        setPower(owner, 0.0, power, MIN_MOTOR_POWER, MAX_MOTOR_POWER, 0.0, null, true);
    }   //setPower

    /**
     * This method sets the PID motor power. It will check for the limit switches. If activated, it won't allow the
     * motor to go in that direction. It will also check for stalled condition and cut motor power if stalled detected.
     * It will also check to reset the stalled condition if reset timeout was specified.
     *
     * @param power specifies the motor power.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power, MIN_MOTOR_POWER, MAX_MOTOR_POWER, 0.0, null, true);
    }   //setPower

    /**
     * This method sets the motor power with PID control. The motor will be under PID control and the power specifies
     * the upper bound of how fast the motor will spin. The actual motor power is controlled by a PID controller with
     * the target either set to minPos or maxPos depending on the direction of the motor. This is very useful in
     * scenarios such as an elevator where you want to have the elevator controlled by a joystick but would like PID
     * control to pay attention to the upper and lower limits and slow down when approaching those limits. The joystick
     * value will specify the upper bound of the elevator power. So if the joystick is only pushed half way, the
     * elevator will only go half power even though it is far away from the target.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setPowerWithinPosRange(String owner, double power, double minPos, double maxPos, boolean holdTarget)
    {
        if (validateOwnership(owner))
        {
            // If power is negative, set the target to minPos. If power is positive, set the target to maxPos. We
            // only set a new target if the target has changed. (i.e. either the motor changes direction, starting
            // or stopping).
            double currPos = getPosition();
            double currTarget = power < 0.0? minPos: power > 0.0? maxPos: minPos - 1.0;
            power = Math.abs(power);
            if (currTarget != taskParams.prevTarget)
            {
                if (power == 0.0)
                {
                    // We are stopping, Relax the power range to max range so we have full power to hold target if
                    // necessary.
                    if (holdTarget)
                    {
                        // Hold target at current position.
                        setPosition(currPos, true, 1.0, null, 0.0);
                    }
                    else
                    {
                        // We reached target and no holding target, we are done.
                        cancel();
                    }
                }
                else
                {
                    // We changed direction, change the target.
                    setPosition(currTarget, holdTarget, power, null, 0.0);
                }
                taskParams.prevTarget = currTarget;
            }
            else if (power == 0.0)
            {
                // We remain stopping, keep the power range relaxed in case we are holding previous target.
                taskParams.outputLimit = 1.0;
            }
            else
            {
                // Direction did not change but we need to update the power range.
                taskParams.outputLimit = power;
            }
        }
    }   //setPowerWithinPosRange

    /**
     * This method sets the motor power with PID control. The motor will be under PID control and the power specifies
     * the upper bound of how fast the motor will spin. The actual motor power is controlled by a PID controller with
     * the target either set to minPos or maxPos depending on the direction of the motor. This is very useful in
     * scenarios such as an elevator where you want to have the elevator controlled by a joystick but would like PID
     * control to pay attention to the upper and lower limits and slow down when approaching those limits. The joystick
     * value will specify the upper bound of the elevator power. So if the joystick is only pushed half way, the
     * elevator will only go half power even though it is far away from the target.
     *
     * @param power specifies the upper bound power of the motor.
     * @param minPos specifies the minimum of the position range.
     * @param maxPos specifies the maximum of the position range.
     * @param holdTarget specifies true to hold target when speed is set to 0, false otherwise.
     */
    public void setPowerWithinPosRange(double power, double minPos, double maxPos, boolean holdTarget)
    {
        setPowerWithinPosRange(null, power, minPos, maxPos, holdTarget);
    }   //setPowerWithinPosRange

    /**
     * This method checks if the PID motor is in the middle of zero calibration.
     *
     * @return true if zero calibrating, false otherwise.
     */
    public boolean isCalibrating()
    {
        return taskParams.calibrating;
    }   //isCalibrating

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     * Generally, calibration power should be negative so that the motor will move towards the lower limit
     * switch. However, in some scenarios such as a turret that can turn all the way around and has only
     * one limit switch, it may be necessary to use a positive calibration power to move it towards the
     * limit switch instead of using negative calibration power and turning the long way around that may
     * cause wires to entangle.
     * If this method specifies an owner and the subsystem was not owned by it, it will acquire exclusive ownership
     * on its behalf and release ownership after the operation is completed.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param calPower specifies the motor power to use for the zero calibration overriding the default calibration
     *                 power specified in the constructor.
     * @param completionEvent specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    public void zeroCalibrate(String owner, double calPower, TrcEvent completionEvent)
    {
        // If a notification event is provided, clear it.
        if (completionEvent != null)
        {
            completionEvent.clear();
        }

        // Caller specifies an owner but has not acquired ownership, let's acquire ownership on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            TrcEvent releaseOwnershipEvent = new TrcEvent(instanceName + ".releaseOwnership");
            OwnershipParams ownershipParams = new OwnershipParams(owner, completionEvent, releaseOwnershipEvent);
            releaseOwnershipEvent.setCallback(this::releaseOwnership, ownershipParams);
            completionEvent = releaseOwnershipEvent;
        }

        if (validateOwnership(owner))
        {
            synchronized (taskParams)
            {
                if (isPidActive())
                {
                    stop(true);
                }
                // In a multiple-motor scenario, motor 1 always has a lower limit switch. If there is a motor 2, motor 2
                // has a lower limit switch only if it is independent of motor 1 and needs synchronizing with motor 1.
                taskParams.calPower = calPower;
                taskParams.notifyEvent = completionEvent;
                taskParams.calibrating = true;
                taskParams.motor1ZeroCalDone = false;
                taskParams.motor2ZeroCalDone = motor2 == null || syncGain == 0.0;
                taskParams.prevTime = TrcTimer.getCurrentTime();
                setTaskEnabled(true);
            }
        }
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param calPower specifies the motor power to use for the zero calibration overriding the default calibration
     *                 power specified in the constructor.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    public void zeroCalibrate(double calPower, TrcEvent event)
    {
        zeroCalibrate(null, calPower, event);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    public void zeroCalibrate(TrcEvent event)
    {
        zeroCalibrate(null, defCalPower, event);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param calPower specifies the motor power to use for the zero calibration overriding the default calibration
     *                 power specified in the constructor.
     * @param callback specifies a callback handler when zero calibration is done.
     * @param callbackContext specifies the context object to pass back to the callback handler.
     */
    public void zeroCalibrate(String owner, double calPower, TrcEvent.Callback callback, Object callbackContext)
    {
        TrcEvent calDoneEvent = new TrcEvent(moduleName + ".calDoneEvent");
        calDoneEvent.setCallback(callback, callbackContext);
        zeroCalibrate(owner, calPower, calDoneEvent);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param calPower specifies the motor power to use for the zero calibration overriding the default calibration
     *                 power specified in the constructor.
     * @param callback specifies a callback handler when zero calibration is done.
     * @param callbackContext specifies the context object to pass back to the callback handler.
     */
    public void zeroCalibrate(double calPower, TrcEvent.Callback callback, Object callbackContext)
    {
        zeroCalibrate(null, calPower, callback, callbackContext);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param callback specifies a callback handler when zero calibration is done.
     * @param callbackContext specifies the context object to pass back to the callback handler.
     */
    public void zeroCalibrate(TrcEvent.Callback callback, Object callbackContext)
    {
        zeroCalibrate(null, defCalPower, callback, callbackContext);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param calPower specifies the motor power to use for the zero calibration overriding the default calibration
     *                 power specified in the constructor.
     */
    public void zeroCalibrate(double calPower)
    {
        zeroCalibrate(null, calPower, null);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        zeroCalibrate(owner, defCalPower, null);
    }   //zeroCalibrate

    /**
     * This method starts zero calibration mode by moving the motor with specified calibration power until a limit
     * switch is hit or the motor is stalled.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null, defCalPower, null);
    }   //zeroCalibrate

    /**
     * This method is called by the calibration task to zero calibrate of a motor. This assumes the caller has
     * acquired the taskParams lock. This method will update currPower and stalled in taskParams.
     *
     * @param motor specifies the motor being calibrated.
     * @param calPower specifies the zero calibration power applied to the motor.
     * @return true if calibration is done, false otherwise.
     */
    private boolean zeroCalibratingMotor(TrcMotor motor, double calPower)
    {
        final String funcName = "zeroCalibratingMotor";
        boolean done = lowerLimitSwitch != null && lowerLimitSwitch.isActive() || taskParams.stalled;

        if (done)
        {
            // Done with zero calibration of the motor. Call the motor directly to stop, do not call any of
            // the setPower or setMotorPower because they do not handle zero calibration mode.
            if (taskParams.stalled)
            {
                if (beepDevice != null)
                {
                    beepDevice.playTone(beepLowFrequency, beepDuration);
                }
                globalTracer.traceWarn(
                    funcName, "%s.%s: Stalled, lower limit switch might have failed!", moduleName, instanceName);
            }
            taskParams.currPower = 0.0;
            taskParams.stalled = false;
            motor.set(0.0);
            motor.resetPosition(false);
        }
        else
        {
            taskParams.currPower = calPower;
            taskParams.stalled = isMotorStalled(calPower);
            motor.set(calPower);
        }

        return done;
    }   //zeroCalibratingMotor

    /**
     * This method performs motor stall detection to protect the motor from burning out. A motor is considered stalled
     * when at least stallMinPower is applied to the motor and the motor is not turning for at least stallTimeout.
     * This assumes the caller has acquired the taskParams lock.
     *
     * @param power specifies the current motor power.
     * @return true if the motor is stalled, false otherwise.
     */
    private boolean isMotorStalled(double power)
    {
        final String funcName = "isMotorStalled";
        boolean stalled = false;

        if (taskParams.stallMinPower > 0.0 && taskParams.stallTimeout > 0.0)
        {
            // Stall protection is ON, check for stall condition.
            // - power is above stallMinPower
            // - motor has not moved for at least stallTimeout.
            double currTime = TrcTimer.getCurrentTime();
            double currPos = getPosition();
            if (Math.abs(power) < taskParams.stallMinPower ||
                Math.abs(currPos - taskParams.prevPos) > taskParams.stallTolerance ||
                taskParams.prevTime == 0.0)
            {
                taskParams.prevPos = currPos;
                taskParams.prevTime = currTime;
            }

            if (currTime - taskParams.prevTime > taskParams.stallTimeout)
            {
                // We have detected a stalled condition for at least stallTimeout. Kill power to protect
                // the motor.
                stalled = true;
                if (beepDevice != null)
                {
                    beepDevice.playTone(beepHighFrequency, beepDuration);
                }

                if (msgTracer != null)
                {
                    msgTracer.traceInfo(funcName, "%s.%s: stalled", moduleName, instanceName);
                }
            }
        }

        return stalled;
    }   //isMotorStalled

    /**
     * This method checks if the motor was stalled and if power has been removed for at least resetTimeout, the
     * stalled condition is then cleared.
     *
     * @param power specifies the power applying to the motor.
     * @return true if the motor was stalled, false otherwise.
     */
    private boolean resetStall(double power)
    {
        boolean wasStalled;

        synchronized (taskParams)
        {
            wasStalled = taskParams.stalled;
            if (wasStalled)
            {
                if (power == 0.0)
                {
                    // We had a stalled condition but if power is removed for at least reset timeout, we clear the
                    // stalled condition.
                    if (taskParams.resetTimeout == 0.0 ||
                        TrcTimer.getCurrentTime() - taskParams.prevTime >= taskParams.resetTimeout)
                    {
                        taskParams.prevPos = getPosition();
                        taskParams.prevTime = TrcTimer.getCurrentTime();
                        taskParams.stalled = false;
                        if (beepDevice != null)
                        {
                            beepDevice.playTone(beepLowFrequency, beepDuration);
                        }
                    }
                }
                else
                {
                    taskParams.prevTime = TrcTimer.getCurrentTime();
                }
            }
        }

        return wasStalled;
    }   //resetStall

    /**
     * This method sets the motor power. If there are two motors, it will set both.
     *
     * @param power specifies the motor power.
     */
    private void setMotorPower(double power)
    {
        final String funcName = "setMotorPower";

        power = TrcUtil.clipRange(power, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
        if (power == 0.0 || syncGain == 0.0)
        {
            // If we are not sync'ing or stopping, just set the motor power. If we are stopping the motor, even if
            // we are sync'ing, we should just stop. But we should still observe the limit switches.
            motor1.set(power);
            if (motor2 != null)
            {
                motor2.set(power);
            }
        }
        else
        {
            // We are sync'ing the two motors and the motor power is not zero.
            double pos1 = motor1.getPosition();
            double pos2 = motor2.getPosition();
            double deltaPower = TrcUtil.clipRange((pos2 - pos1)*syncGain);
            double power1 = power + deltaPower;
            double power2 = power - deltaPower;
            double minPower = Math.min(power1, power2);
            double maxPower = Math.max(power1, power2);
            double scale = maxPower > MAX_MOTOR_POWER? maxPower: minPower < MIN_MOTOR_POWER? -minPower: 1.0;
            // We don't want the motors to switch direction in order to sync. It will worsen oscillation.
            // So make sure the motor powers are moving in the same direction.
            if (power > 0.0)
            {
                power1 = TrcUtil.clipRange(power1/scale, 0.0, MAX_MOTOR_POWER);
                power2 = TrcUtil.clipRange(power2/scale, 0.0, MAX_MOTOR_POWER);
            }
            else
            {
                power1 = TrcUtil.clipRange(power1, MIN_MOTOR_POWER, 0.0);
                power2 = TrcUtil.clipRange(power2, MIN_MOTOR_POWER, 0.0);
            }

            motor1.set(power1);
            motor2.set(power2);

            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.%s: power=%.2f,deltaPower=%.2f,pos1=%.0f,pos2=%.0f,power1=%.2f,power2=%.2f",
                    moduleName, instanceName, power, deltaPower, pos1, pos2, power1, power2);
            }
        }
    }   //setMotorPower

    /**
     * This method activates/deactivates a PID motor operation by enabling/disabling the PID motor task.
     *
     * @param enabled specifies true to activate a PID motor operation, false otherwise.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            pidMotorTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
            stopMotorTaskObj.registerTask(TrcTaskMgr.TaskType.STOP_TASK);
        }
        else
        {
            pidMotorTaskObj.unregisterTask();
            stopMotorTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method is called periodically to perform the PID motor task. The PID motor task can be in one of two
     * modes: zero calibration mode and normal mode. In zero calibration mode, it will drive the motor with the
     * specified calibration power until it hits the lower limit switch or the motor is stalled. Then it will stop
     * the motor and reset the motor position sensor. In normal mode, it calls the PID control to calculate and set
     * the motor power. It also checks if the motor has reached the set target and disables the task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void pidMotorTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = "pidMotorTask";
        TrcEvent completionEvent = null;
        boolean done = false;

        synchronized (taskParams)
        {
            if (taskParams.calibrating)
            {
                // We are in zero calibration mode.
                if (!taskParams.motor1ZeroCalDone)
                {
                    taskParams.motor1ZeroCalDone = zeroCalibratingMotor(motor1, taskParams.calPower);
                }

                if (!taskParams.motor2ZeroCalDone)
                {
                    taskParams.motor2ZeroCalDone = zeroCalibratingMotor(motor2, taskParams.calPower);
                }

                if (taskParams.motor1ZeroCalDone && taskParams.motor2ZeroCalDone)
                {
                    // Done with zero calibration.
                    taskParams.calibrating = false;
                    completionEvent = taskParams.notifyEvent;
                    taskParams.notifyEvent = null;
                    done = true;
                }
            }
            else
            {
                // Perform our software PID control on the motor.
                boolean onTarget = pidCtrl != null?
                    pidCtrl.isOnTarget(): Math.abs(taskParams.currTarget - getPosition()) <= pidParams.tolerance;
                boolean expired = taskParams.timeout != 0.0 && TrcTimer.getCurrentTime() >= taskParams.timeout;
                boolean doStop = taskParams.stalled || !taskParams.holdTarget && (onTarget || expired);

                if (doStop)
                {
                    stop(true);
                }
                else if (pidCtrl != null)
                {
                    // Doing software PID control here. If we are using motor close loop control, there is no action.
                    // We just monitor the control progress.
                    performSetPower(new SetPowerParams(
                        pidCtrl.getOutput(), -taskParams.outputLimit, taskParams.outputLimit, 0.0, null, false));
                    if (msgTracer != null && tracePidInfo)
                    {
                        pidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
                    }
                }

                if (taskParams.stalled || expired || onTarget)
                {
                    // We signal done if either we:
                    // - are stalled.
                    // - set a timeout and it has expired.
                    // - have reached target.
                    completionEvent = taskParams.notifyEvent;
                    taskParams.notifyEvent = null;
                    if (completionEvent != null && msgTracer != null)
                    {
                        msgTracer.traceInfo(
                            funcName, "%s.%s: stalled=%s, expired=%s, onTarget=%s, event=%s",
                            moduleName, instanceName, taskParams.stalled, expired, onTarget, completionEvent);
                        pidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
                    }
                }
            }
        }

        if (completionEvent != null)
        {
            completionEvent.signal();
        }

        if (done)
        {
            setTaskEnabled(false);
        }
    }   //pidMotorTask

    /**
     * This method is called when the competition mode is about to end to stop the PID motor operation if any.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void stopMotorTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        stop(true);
    }   //stopMotorTask

}   //class TrcPidMotor
