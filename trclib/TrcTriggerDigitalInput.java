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

import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This class implements a trigger for a digital input device. A digital input trigger consists of a digital input
 * device. It monitors the device state and notifies the callback handler if the state changes.
 */
public class TrcTriggerDigitalInput implements TrcTrigger
{
    private static final String moduleName = "TrcTriggerDigitInput";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    /**
     * This class encapsulates the trigger state. Access to this object must be thread safe (i.e. needs to be
     * synchronized).
     */
    private static class TriggerState
    {
        volatile boolean sensorState;
        volatile boolean triggerEnabled;

        TriggerState(boolean state, boolean enabled)
        {
            this.sensorState = state;
            this.triggerEnabled = enabled;
        }   //TriggerState

        @Override
        public String toString()
        {
            return String.format(Locale.US, "(state=%s,enabled=%s)", sensorState, triggerEnabled);
        }   //toString

    }   //class TriggerState

    private final String instanceName;
    private final TrcDigitalInput sensor;
    private final TrcEvent.Callback triggerCallback;
    private final Thread callbackThread;
    private final TriggerState triggerState;
    private final TrcEvent callbackEvent;
    private final AtomicBoolean callbackContext;
    private final TrcTaskMgr.TaskObject triggerTaskObj;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the digital input device.
     * @param triggerCallback specifies the callback handler to notify when the trigger state changed.
     */
    public TrcTriggerDigitalInput(
        String instanceName, TrcDigitalInput sensor, TrcEvent.Callback triggerCallback)
    {
        if (sensor == null || triggerCallback == null)
        {
            throw new IllegalArgumentException("Sensor/TriggerCallback cannot be null.");
        }

        this.instanceName = instanceName;
        this.sensor = sensor;
        this.triggerCallback = triggerCallback;
        callbackThread = Thread.currentThread();

        triggerState = new TriggerState(sensor.isActive(), false);
        callbackEvent = new TrcEvent(instanceName + ".callbackEvent");
        callbackContext = new AtomicBoolean();
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcTriggerDigitalInput

    /**
     * This method returns the instance name and its state.
     *
     * @return instance name and state.
     */
    @Override
    public String toString()
    {
        String str;

        synchronized (triggerState)
        {
            str = String.format(Locale.US, "%s.%s=%s", moduleName, instanceName, triggerState);
        }

        return str;
    }   //toString

    //
    // Implements TrcTrigger interface.
    //

    /**
     * This method enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    @Override
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        synchronized (triggerState)
        {
            if (enabled)
            {
                triggerState.sensorState = sensor.isActive();
                triggerTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
            }
            else
            {
                triggerTaskObj.unregisterTask();
            }
            triggerState.triggerEnabled = enabled;

            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.%s: enabled=%s (state=%s)", moduleName, instanceName, enabled, triggerState);
            }
        }
    }   //setEnabled

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    @Override
    public boolean isEnabled()
    {
        return triggerState.triggerEnabled;
    }   //isEnabled

    /**
     * This method reads the current analog sensor value (not supported).
     *
     * @return current sensor value, null if it failed to read the sensor.
     */
    @Override
    public double getSensorValue()
    {
        throw new UnsupportedOperationException("Digital sensor does not support analog value.");
    }   //getSensorValue

    /**
     * This method reads the current digital sensor state.
     *
     * @return current sensor state.
     */
    @Override
    public boolean getSensorState()
    {
        return sensor.isActive();
    }   //getSensorState

    /**
     * This method is called periodically to check the current sensor state. If it has changed from the previous
     * state, the triggerCallback will be notified.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = "triggerTask";
        boolean currState = getSensorState();
        boolean triggered = false;
        boolean prevState = false;

        synchronized (triggerState)
        {
            if (currState != triggerState.sensorState)
            {
                prevState = triggerState.sensorState;
                triggerState.sensorState = currState;
                triggered = true;
            }
        }

        if (triggered)
        {
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.%s: changes state %s->%s", moduleName, instanceName, prevState, currState);
            }

            callbackContext.set(currState);
            callbackEvent.setCallback(callbackThread, triggerCallback, callbackContext);
            callbackEvent.signal();
        }
    }   //triggerTask

}   //class TrcTriggerDigitalInput
