/**
 * Hook for tracking reading progress with heartbeat functionality.
 */

import { useEffect, useRef, useCallback } from 'react';
import { progressService } from '../services/progress';
import { useAuth } from '../context/AuthContext';

const HEARTBEAT_INTERVAL = 30000; // 30 seconds

interface UseReadingProgressOptions {
  chapterId: string;
  enabled?: boolean;
}

export function useReadingProgress({
  chapterId,
  enabled = true,
}: UseReadingProgressOptions): void {
  const { user } = useAuth();
  const lastHeartbeatRef = useRef<number>(Date.now());
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const sendHeartbeat = useCallback(async () => {
    if (!user || !enabled) return;

    const now = Date.now();
    const durationSeconds = Math.round((now - lastHeartbeatRef.current) / 1000);
    lastHeartbeatRef.current = now;

    // Get current scroll position
    const position = typeof window !== 'undefined'
      ? `${Math.round(window.scrollY)}px`
      : undefined;

    try {
      await progressService.heartbeat(chapterId, position, durationSeconds);
    } catch (error) {
      // Silently fail - don't interrupt reading
      console.debug('Failed to send heartbeat:', error);
    }
  }, [chapterId, user, enabled]);

  useEffect(() => {
    if (!enabled || !user) {
      return;
    }

    // Send initial heartbeat
    sendHeartbeat();

    // Set up interval
    intervalRef.current = setInterval(sendHeartbeat, HEARTBEAT_INTERVAL);

    // Send heartbeat on visibility change (when user comes back to tab)
    const handleVisibilityChange = () => {
      if (document.visibilityState === 'visible') {
        lastHeartbeatRef.current = Date.now();
      } else {
        sendHeartbeat();
      }
    };

    document.addEventListener('visibilitychange', handleVisibilityChange);

    // Send heartbeat before unload
    const handleBeforeUnload = () => {
      sendHeartbeat();
    };

    window.addEventListener('beforeunload', handleBeforeUnload);

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
      document.removeEventListener('visibilitychange', handleVisibilityChange);
      window.removeEventListener('beforeunload', handleBeforeUnload);

      // Send final heartbeat
      sendHeartbeat();
    };
  }, [chapterId, user, enabled, sendHeartbeat]);
}

export default useReadingProgress;
