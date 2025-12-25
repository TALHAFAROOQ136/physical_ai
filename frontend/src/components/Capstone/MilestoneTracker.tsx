/**
 * Milestone tracker component for capstone project progress.
 * Shows progress through capstone milestones with completion status.
 */

import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '../../context/AuthContext';
import { progressService, ChapterProgress } from '../../services/progress';
import styles from './Capstone.module.css';

interface Milestone {
  id: string;
  title: string;
  description: string;
  chapterId: string;
  order: number;
}

const milestones: Milestone[] = [
  {
    id: 'milestone-1',
    title: 'ROS 2 Setup',
    description: 'Set up your ROS 2 development environment and create your first package',
    chapterId: 'capstone-milestone-1',
    order: 1,
  },
  {
    id: 'milestone-2',
    title: 'Gazebo Simulation',
    description: 'Build a humanoid robot model and simulate it in Gazebo',
    chapterId: 'capstone-milestone-2',
    order: 2,
  },
  {
    id: 'milestone-3',
    title: 'Isaac Integration',
    description: 'Integrate NVIDIA Isaac Sim for advanced perception',
    chapterId: 'capstone-milestone-3',
    order: 3,
  },
  {
    id: 'milestone-4',
    title: 'Voice Interface',
    description: 'Add natural language voice control using LLMs',
    chapterId: 'capstone-milestone-4',
    order: 4,
  },
  {
    id: 'milestone-5',
    title: 'Final Integration',
    description: 'Combine all systems into a complete humanoid robot',
    chapterId: 'capstone-milestone-5',
    order: 5,
  },
];

interface MilestoneTrackerProps {
  currentMilestone?: string;
}

export function MilestoneTracker({
  currentMilestone,
}: MilestoneTrackerProps): JSX.Element {
  const { user, isLoading: authLoading } = useAuth();
  const [progress, setProgress] = useState<Record<string, ChapterProgress>>({});
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (!user || authLoading) {
      setLoading(false);
      return;
    }

    // Fetch progress for all milestone chapters
    Promise.all(
      milestones.map((m) =>
        progressService
          .getChapterProgress(m.chapterId)
          .catch(() => null)
      )
    )
      .then((results) => {
        const progressMap: Record<string, ChapterProgress> = {};
        results.forEach((result, index) => {
          if (result) {
            progressMap[milestones[index].chapterId] = result;
          }
        });
        setProgress(progressMap);
      })
      .finally(() => {
        setLoading(false);
      });
  }, [user, authLoading]);

  const getMilestoneStatus = (milestone: Milestone) => {
    const chapterProgress = progress[milestone.chapterId];
    if (!chapterProgress) return 'not_started';
    return chapterProgress.status;
  };

  const getCompletedCount = () => {
    return milestones.filter(
      (m) => getMilestoneStatus(m) === 'completed'
    ).length;
  };

  return (
    <div className={styles.tracker}>
      <div className={styles.trackerHeader}>
        <h3 className={styles.trackerTitle}>Capstone Progress</h3>
        <span className={styles.trackerProgress}>
          {getCompletedCount()} / {milestones.length} milestones
        </span>
      </div>

      <div className={styles.milestoneList}>
        {milestones.map((milestone, index) => {
          const status = getMilestoneStatus(milestone);
          const isCurrent = milestone.id === currentMilestone;
          const isCompleted = status === 'completed';
          const isInProgress = status === 'in_progress';

          return (
            <div
              key={milestone.id}
              className={`${styles.milestoneItem} ${
                isCurrent ? styles.current : ''
              } ${isCompleted ? styles.completed : ''} ${
                isInProgress ? styles.inProgress : ''
              }`}
            >
              <div className={styles.milestoneConnector}>
                <div className={styles.milestoneNumber}>
                  {isCompleted ? '✓' : index + 1}
                </div>
                {index < milestones.length - 1 && (
                  <div
                    className={`${styles.milestoneLine} ${
                      isCompleted ? styles.completedLine : ''
                    }`}
                  />
                )}
              </div>
              <div className={styles.milestoneContent}>
                <Link
                  to={`/docs/capstone/${milestone.id}`}
                  className={styles.milestoneTitle}
                >
                  {milestone.title}
                </Link>
                <p className={styles.milestoneDescription}>
                  {milestone.description}
                </p>
                {isCompleted && (
                  <span className={styles.statusBadge}>Completed</span>
                )}
                {isInProgress && (
                  <span className={`${styles.statusBadge} ${styles.inProgressBadge}`}>
                    In Progress
                  </span>
                )}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}

export default MilestoneTracker;
