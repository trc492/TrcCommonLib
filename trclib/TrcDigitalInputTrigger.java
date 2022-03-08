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
 * This class implements a trigger for a digital input device. A digital input trigger consists of a digital input
 * device. It monitors the device state and calls the notification handler if the state changes.
 */
public class TrcDigitalInputTrigger extends TrcTrigger
{
    private final TrcDigitalInput sensor;
    private final DigitalTriggerHandler triggerHandler;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private boolean sensorState;
    private boolean enabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the digital input device.
     * @param triggerHandler specifies the object that will be called to handle the digital input device state change.
     */
    public TrcDigitalInputTrigger(String instanceName, TrcDigitalInput sensor, DigitalTriggerHandler triggerHandler)
    {
        super(instanceName);

        if (sensor == null || triggerHandler == null)
        {
            throw new IllegalArgumentException("Sensor/TriggerHandler cannot be null.");
        }

        this.sensor = sensor;
        this.triggerHandler = triggerHandler;
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);

        sensorState = sensor.isActive();
    }   //TrcDigitalInputTrigger

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
            sensorState = sensor.isActive();
            triggerTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            triggerTaskObj.unregisterTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
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
    public boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method reads the current analog sensor value (not supported).
     *
     * @return current sensor value, null if it failed to read the sensor.
     */
    @Override
    public double getValue()
    {
        throw new UnsupportedOperationException("Digital sensor does not support analog value.");
    }   //getSensorValue

    /**
     * This method reads the current digital sensor state.
     *
     * @return current sensor state.
     */
    @Override
    public boolean getState()
    {
        return sensor.isActive();
    }   //getSensorState

    /**
     * This method is called periodically to check the current sensor state. If it has changed from the previous
     * state, the triggerHandler will be notified.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "triggerTask";
        boolean currState = getState();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (currState != sensorState)
        {
            if (triggerHandler != null)
            {
                triggerHandler.digitalTriggerEvent(currState);
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "%s changes state %s->%s", instanceName, sensorState, currState);
            }
        }
        sensorState = currState;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //triggerTask

}   //class TrcDigitalInputTrigger
