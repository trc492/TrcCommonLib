/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements a ValueSensorTrigger. It monitors the sensor value against the lower and upper threshold
 * values. If the sensor value stays within the lower and upper threshold for at least the given settling period,
 * the trigger state is set to active and the trigger handler is called. If the sensor value exits the threshold
 * range, the trigger state is set to inactive and the trigger handler is also called.
 */
public class TrcValueSensorTrigger extends TrcSensorTrigger
{
    /**
     * This interface implements the method to read the sensor value.
     */
    public interface ValueSource
    {
        /**
         * This method reads the sensor value.
         *
         * @return sensor value.
         */
        double getValue();
    }   //interface ValueSource

    private final ValueSource valueSource;
    private final DigitalTriggerHandler triggerHandler;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private double lowerThreshold, upperThreshold, settlingPeriod;
    private boolean enabled = false;
    private boolean triggerActive = false;
    private double startTime;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSource specifies the interface that implements the value source.
     * @param triggerHandler specifies the object to handle the trigger event.
     */
    public TrcValueSensorTrigger(String instanceName, ValueSource valueSource, DigitalTriggerHandler triggerHandler)
    {
        super(instanceName);

        if (valueSource == null || triggerHandler == null)
        {
            throw new NullPointerException("ValueSource/TriggerHandler cannot be null.");
        }

        this.valueSource = valueSource;
        this.triggerHandler = triggerHandler;
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcValueSensorTrigger

    /**
     * This method sets the lower/upper threshold values within which the sensor reading must stay for at least the
     * settling period for it to trigger the notification.
     *
     * @param lowerThreshold specifies the lower threshold value for the trigger.
     * @param upperThreshold specifies the upper threshold value for the trigger.
     * @param settlingPeriod specifies the period in seconds the sensor value must stay within threshold range for it
     *                       to trigger.
     */
    public synchronized void setTrigger(double lowerThreshold, double upperThreshold, double settlingPeriod)
    {
        final String funcName = "setTrigger";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "lowerThreshold=%f,upperThreshold=%f,settingPeriod=%.3f",
                lowerThreshold, upperThreshold, settlingPeriod);
        }

        this.lowerThreshold = lowerThreshold;
        this.upperThreshold = upperThreshold;
        this.settlingPeriod = settlingPeriod;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTrigger

    //
    // Implements TrcSensorTrigger abstract methods.
    //

    /**
     * This method enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    @Override
    public synchronized void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (enabled)
        {
            startTime = TrcUtil.getCurrentTime();
            triggerTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            triggerTaskObj.unregisterTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        triggerActive = false;
        this.enabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    @Override
    public synchronized boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method reads the current analog sensor value. It may return null if it failed to read the sensor.
     *
     * @return current sensor value, null if it failed to read the sensor.
     */
    @Override
    public double getSensorValue()
    {
        return valueSource.getValue();
    }   //getSensorValue

    /**
     * This method reads the current digital sensor state (not supported).
     *
     * @return current sensor state.
     */
    @Override
    public boolean getSensorState()
    {
        return triggerActive;
    }   //getSensorState

    /**
     * This method is called periodically to check if the sensor value is within the lower and upper threshold range.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "triggerTask";
        double currValue = valueSource.getValue();
        double currTime = TrcUtil.getCurrentTime();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (currValue < lowerThreshold || currValue > upperThreshold)
        {
            if (triggerActive)
            {
                // Only fires a trigger event when the threshold range is first exited (i.e. edge event).
                triggerActive = false;
                triggerHandler.digitalTriggerEvent(triggerActive);
            }
            startTime = currTime;
        }
        else if (currTime >= startTime + settlingPeriod)
        {
            if (!triggerActive)
            {
                // Only fires a trigger event when the threshold range is first entered and stayed for settling period
                // (i.e. edge event).
                triggerActive = true;
                triggerHandler.digitalTriggerEvent(triggerActive);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //triggerTask

}   //class TrcValueSensorTrigger
