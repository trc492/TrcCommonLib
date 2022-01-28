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
import java.util.HashMap;

/**
 * This class implements the TrcTimer manager that uses a single thread to monitor all TrcTimers.
 */
public class TrcTimerMgr
{
    private static final String moduleName = "TrcTimerMgr";
    private static final boolean debugEnabled = false;
    private static TrcDbgTrace dbgTrace = TrcDbgTrace.getGlobalTracer();

    private static final ArrayList<TrcTimer> timerList = new ArrayList<>();
    private static final HashMap<TrcTimer, Double> securityKeyMap = new HashMap<>();
    private static Thread timerThread = null;
    private static boolean shuttingDown = false;
    private static TrcTimer nextTimerToExpire = null;
    private static TrcTimer preemptingTimer = null;

    /**
     * This method is called by the TrcTaskMgr to shut down TrcTimerMgr when it is exiting.
     */
    public static void shutdown()
    {
        if (timerThread != null)
        {
            shuttingDown = true;
            timerThread.interrupt();
        }
    }   //shutdown

    /**
     * This method adds the timer to the timer list in the order of expiration.
     *
     * @param timer specifies the timer to be added to the list.
     * @param callerID specifies the security token for identifying the caller.
     * @return securityKey that combines the caller's ID and a unique identifier assigned by TrcTimerMgr to the timer.
     */
    public static double add(TrcTimer timer, double callerID)
    {
        final String funcName = "add";
        double securityKey;
        long expiredTimeInMsec = timer.getExpiredTimeInMsec();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "timer=%s,callerID=%f", timer, callerID);
        }

        synchronized (timerList)
        {
            int position = -1;

            if (nextTimerToExpire != null && expiredTimeInMsec < nextTimerToExpire.getExpiredTimeInMsec())
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "[%d] Adding timer %s preempting %s.",
                        TrcUtil.getCurrentTimeMillis(), timer, nextTimerToExpire);
                }
                //
                // The added new timer expires sooner than the one we are sleeping on. Let's interrupt its sleep and
                // process this one first.
                //
                preemptingTimer = timer;
                timerThread.interrupt();
                //
                // The final security key is the sum of the list position and the caller's identification key.
                // Since this timer is not added to the list, the position will be -1.
                //
                securityKey = position + callerID;
                securityKeyMap.put(timer, securityKey);
            }
            else
            {
                for (int i = 0; i < timerList.size(); i++)
                {
                    if (expiredTimeInMsec < timerList.get(i).getExpiredTimeInMsec())
                    {
                        position = i;
                        break;
                    }
                }

                if (position == -1)
                {
                    //
                    // This must be the last in the list or the only one in the list, add it to the end.
                    //
                    position = timerList.size();
                }

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(
                        funcName, "[%d] Adding timer %s to queue position %d.",
                        TrcUtil.getCurrentTimeMillis(), timer, position);
                }
                timerList.add(position, timer);
                //
                // The final security key is the sum of the list position and the caller's identification key.
                //
                securityKey = position + callerID;
                securityKeyMap.put(timer, position + callerID);
                //
                // In case this is the first and only timer in the list, kick start the timer thread.
                //
                timerList.notify();
            }
        }

        if (timerThread == null)
        {
            // Timer thread does not exist, let's create one and start it.
            timerThread = new Thread(TrcTimerMgr::timerTask, moduleName);
            timerThread.start();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", securityKey);
            dbgTrace.traceInfo(funcName, "Adding timer %s, securityKey=%f.", timer, securityKey);
        }

        return securityKey;
    }   //add

    /**
     * This method removes a timer from the list.
     *
     * @param timer specifies the timer to be removed.
     * @param securityKey specifies the security key identifying the owner of the timer to be removed.
     * @return true if the timer is removed, false otherwise.
     */
    public static boolean remove(TrcTimer timer, double securityKey)
    {
        final String funcName = "remove";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "timer=%s,securityKey=%f", timer, securityKey);
            dbgTrace.traceInfo(funcName, "Removing timer %s.", timer);
        }

        //
        // Only do this if we are not shutting down. If we are shutting down, thread cleanup will take care of
        // the timer list.
        //
        if (!shuttingDown)
        {
            synchronized (timerList)
            {
                Double key = securityKeyMap.get(timer);

                if (key != null)
                {
                    if (securityKey == key)
                    {
                        if (timer == nextTimerToExpire)
                        {
                            timerThread.interrupt();
                            success = true;
                        }
                        else
                        {
                            success = timerList.remove(timer);
                        }
                    }
                    else
                    {
                        throw new SecurityException("Only the owner of the timer is allowed to remove it from the list.");
                    }
                }
                else
                {
                    dbgTrace.traceWarn(funcName, "Timer %s not found in security key map.");
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", success);
            dbgTrace.traceInfo(funcName, "timer=%s,securityKey=%f,timerSecurity=%f",
                    timer, securityKey, securityKeyMap.get(timer));
        }

        return success;
    }   //remove

    /**
     * This method runs by the timer thread to wait for the next timer in the list and signal the timer object when
     * it expires.
     */
    private static void timerTask()
    {
        final String funcName = "timerTask";

        if (debugEnabled)
        {
            dbgTrace.traceInfo("%s is starting...", moduleName);
        }

        while (!shuttingDown)
        {
            try
            {
                long sleepTimeInMsec;

                synchronized (timerList)
                {
                    if (nextTimerToExpire == null && timerList.isEmpty())
                    {
                        //
                        // No more timer to process, let's wait.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Waiting for timer...");
                        }
                        timerList.wait();
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "New timer arrived.");
                        }
                    }

                    if (nextTimerToExpire == null)
                    {
                        // There is no timer in progress, get one from the timer queue.
                        nextTimerToExpire = timerList.remove(0);
                    }

                    sleepTimeInMsec = nextTimerToExpire != null?
                        nextTimerToExpire.getExpiredTimeInMsec() - TrcUtil.getCurrentTimeMillis(): 0;
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "[%d]: timer=%s, sleepTimeInMsec=%d",
                            TrcUtil.getCurrentTimeMillis(), nextTimerToExpire, sleepTimeInMsec);
                    }
                }

                if (sleepTimeInMsec > 0)
                {
                    Thread.sleep(sleepTimeInMsec);
                }

                if (nextTimerToExpire != null)
                {
                    //
                    // Timer has expired, signal it.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(
                            funcName, "[%d]: timer %s expired.", TrcUtil.getCurrentTimeMillis(), nextTimerToExpire);
                    }

                    nextTimerToExpire.setExpired(securityKeyMap.get(nextTimerToExpire));
                    nextTimerToExpire = null;
                }
            }
            catch (InterruptedException e)
            {
                synchronized (timerList)
                {
                    if (preemptingTimer != null)
                    {
                        if (nextTimerToExpire != null)
                        {
                            //
                            // Somebody just added a timer that will expire sooner than the one we are sleeping on.
                            // Push this timer back to the front of the list and continue the next loop so the
                            // preempting timer will be processed first. If nextTimerToExpire is null, it means
                            // the current timer has been canceled, so don't put it back in the timer queue.
                            //
                            timerList.add(0, nextTimerToExpire);
                        }

                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(
                                funcName, "[%d] Timer %s is preempting %s.",
                                TrcUtil.getCurrentTimeMillis(), preemptingTimer, nextTimerToExpire);
                        }

                        nextTimerToExpire = preemptingTimer;
                        preemptingTimer = null;
                    }
                    else if (nextTimerToExpire != null && nextTimerToExpire.isCanceled())
                    {
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(
                                funcName, "[%d] timer %s was canceled.",
                                TrcUtil.getCurrentTimeMillis(), nextTimerToExpire);
                        }
                        nextTimerToExpire = null;
                    }
                    else if (shuttingDown)
                    {
                        //
                        // Somebody is trying to terminate the timer thread. Let's quit.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Terminating %s", moduleName);
                        }
                        break;
                    }
                }
            }
        }
        //
        // The thread is terminating, cancel all pending timers before exiting.
        //
        synchronized (timerList)
        {
            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "Terminating: canceling %d timers", timerList.size());
            }

            for (TrcTimer timer: timerList)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "Canceling %s", timer);
                }
                timer.cancel();
            }

            timerList.clear();
        }

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "%s is terminated", moduleName);
        }
        //
        // The thread is now terminated. Destroy this instance so we will recreate the thread the next time around.
        //
        timerThread = null;
        shuttingDown = false;
    }   //timerTask

}   //class TrcTimerMgr
