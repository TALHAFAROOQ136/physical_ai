/**
 * Progress tracking service.
 *
 * DISABLED MODE: Returns mock data when backend is unavailable.
 * Progress is not persisted without a backend.
 */

import { api } from './api';

// Types
export interface ChapterProgress {
  chapterId: string;
  moduleId: string;
  status: 'not_started' | 'in_progress' | 'completed';
  completedAt: string | null;
  readingTimeSeconds: number;
  lastPosition: string | null;
  updatedAt: string;
}

export interface ProgressSummary {
  totalChapters: number;
  completedChapters: number;
  inProgressChapters: number;
  totalReadingTimeMinutes: number;
  chapters: ChapterProgress[];
}

export interface OverallProgress {
  completed: number;
  inProgress: number;
  notStarted: number;
  totalChapters: number;
  percentComplete: number;
}

export interface ModuleProgress {
  moduleId: string;
  moduleTitle: string;
  completed: number;
  total: number;
  percentComplete: number;
}

export interface RecentActivity {
  chapterId: string;
  chapterTitle: string;
  action: 'started' | 'continued' | 'completed';
  timestamp: string;
}

export interface Dashboard {
  overallProgress: OverallProgress;
  moduleProgress: ModuleProgress[];
  recentActivity: RecentActivity[];
  totalReadingTimeMinutes: number;
  currentStreak: number;
}

export interface NextChapter {
  chapterId: string;
  title: string;
  moduleId: string;
  reason: string;
}

export interface SkipSuggestion {
  chapterId: string;
  title: string;
  reason: string;
}

export interface FocusArea {
  moduleId: string;
  moduleTitle: string;
  priority: 'high' | 'medium' | 'low';
}

export interface Recommendations {
  nextChapter: NextChapter | null;
  skipSuggestions: SkipSuggestion[];
  focusAreas: FocusArea[];
}

// Mock data for when backend is unavailable
const mockDashboard: Dashboard = {
  overallProgress: {
    completed: 0,
    inProgress: 0,
    notStarted: 20,
    totalChapters: 20,
    percentComplete: 0,
  },
  moduleProgress: [
    { moduleId: 'intro', moduleTitle: 'Introduction', completed: 0, total: 3, percentComplete: 0 },
    { moduleId: 'module-1-ros2', moduleTitle: 'Module 1: ROS 2', completed: 0, total: 4, percentComplete: 0 },
    { moduleId: 'module-2-simulation', moduleTitle: 'Module 2: Simulation', completed: 0, total: 4, percentComplete: 0 },
    { moduleId: 'module-3-isaac', moduleTitle: 'Module 3: NVIDIA Isaac', completed: 0, total: 4, percentComplete: 0 },
    { moduleId: 'module-4-vla', moduleTitle: 'Module 4: VLA', completed: 0, total: 4, percentComplete: 0 },
  ],
  recentActivity: [],
  totalReadingTimeMinutes: 0,
  currentStreak: 0,
};

const mockRecommendations: Recommendations = {
  nextChapter: {
    chapterId: 'intro',
    title: 'Introduction to Physical AI',
    moduleId: 'intro',
    reason: 'Start your learning journey here!',
  },
  skipSuggestions: [],
  focusAreas: [],
};

const mockProgressSummary: ProgressSummary = {
  totalChapters: 20,
  completedChapters: 0,
  inProgressChapters: 0,
  totalReadingTimeMinutes: 0,
  chapters: [],
};

const createMockChapterProgress = (chapterId: string): ChapterProgress => ({
  chapterId,
  moduleId: 'unknown',
  status: 'not_started',
  completedAt: null,
  readingTimeSeconds: 0,
  lastPosition: null,
  updatedAt: new Date().toISOString(),
});

class ProgressService {
  private backendDisabled = false;

  /**
   * Check if backend is available and cache result.
   */
  private async checkBackend(): Promise<boolean> {
    if (this.backendDisabled) return false;

    try {
      const available = await api.isBackendAvailable();
      if (!available) {
        this.backendDisabled = true;
        console.info('Progress tracking disabled: backend not available');
      }
      return available;
    } catch {
      this.backendDisabled = true;
      return false;
    }
  }

  /**
   * Get all progress for current user.
   */
  async getAllProgress(moduleId?: string): Promise<ProgressSummary> {
    if (!(await this.checkBackend())) {
      return mockProgressSummary;
    }

    try {
      const params = moduleId ? `?module_id=${moduleId}` : '';
      return await api.get<ProgressSummary>(`/progress${params}`);
    } catch {
      return mockProgressSummary;
    }
  }

  /**
   * Get progress for a specific chapter.
   */
  async getChapterProgress(chapterId: string): Promise<ChapterProgress> {
    if (!(await this.checkBackend())) {
      return createMockChapterProgress(chapterId);
    }

    try {
      return await api.get<ChapterProgress>(`/progress/chapters/${chapterId}`);
    } catch {
      return createMockChapterProgress(chapterId);
    }
  }

  /**
   * Update chapter progress.
   */
  async updateChapterProgress(
    chapterId: string,
    data: {
      status?: 'in_progress' | 'completed';
      lastPosition?: string;
      additionalReadingTime?: number;
    }
  ): Promise<ChapterProgress> {
    if (!(await this.checkBackend())) {
      // Return mock updated progress
      return {
        ...createMockChapterProgress(chapterId),
        status: data.status || 'not_started',
        lastPosition: data.lastPosition || null,
      };
    }

    try {
      return await api.patch<ChapterProgress>(`/progress/chapters/${chapterId}`, {
        status: data.status,
        last_position: data.lastPosition,
        additional_reading_time: data.additionalReadingTime,
      });
    } catch {
      return createMockChapterProgress(chapterId);
    }
  }

  /**
   * Mark a chapter as complete.
   */
  async markChapterComplete(chapterId: string): Promise<ChapterProgress> {
    if (!(await this.checkBackend())) {
      return {
        ...createMockChapterProgress(chapterId),
        status: 'completed',
        completedAt: new Date().toISOString(),
      };
    }

    try {
      return await api.post<ChapterProgress>(`/progress/chapters/${chapterId}/complete`);
    } catch {
      return createMockChapterProgress(chapterId);
    }
  }

  /**
   * Unmark chapter completion.
   */
  async unmarkChapterComplete(chapterId: string): Promise<ChapterProgress> {
    if (!(await this.checkBackend())) {
      return createMockChapterProgress(chapterId);
    }

    try {
      return await api.delete<ChapterProgress>(`/progress/chapters/${chapterId}/complete`);
    } catch {
      return createMockChapterProgress(chapterId);
    }
  }

  /**
   * Get dashboard data.
   */
  async getDashboard(): Promise<Dashboard> {
    if (!(await this.checkBackend())) {
      return mockDashboard;
    }

    try {
      return await api.get<Dashboard>('/progress/dashboard');
    } catch {
      return mockDashboard;
    }
  }

  /**
   * Get recommendations.
   */
  async getRecommendations(): Promise<Recommendations> {
    if (!(await this.checkBackend())) {
      return mockRecommendations;
    }

    try {
      return await api.get<Recommendations>('/progress/recommendations');
    } catch {
      return mockRecommendations;
    }
  }

  /**
   * Send reading heartbeat.
   */
  async heartbeat(
    chapterId: string,
    position?: string,
    durationSeconds: number = 30
  ): Promise<void> {
    if (!(await this.checkBackend())) {
      // Silently ignore heartbeats when backend is unavailable
      return;
    }

    try {
      await api.post('/progress/heartbeat', {
        chapter_id: chapterId,
        position,
        duration_seconds: durationSeconds,
      });
    } catch {
      // Silently fail heartbeats
    }
  }
}

// Export singleton instance
export const progressService = new ProgressService();

// Export class for testing
export { ProgressService };
