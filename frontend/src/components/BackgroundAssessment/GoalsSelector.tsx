/**
 * Learning goals selector component for background assessment.
 */

import React from 'react';
import styles from './BackgroundAssessment.module.css';

interface Goal {
  id: string;
  label: string;
  description: string;
  icon: string;
}

interface GoalsSelectorProps {
  selectedGoals: string[];
  onChange: (goals: string[]) => void;
}

const availableGoals: Goal[] = [
  {
    id: 'ros2',
    label: 'ROS 2 Fundamentals',
    description: 'Learn the robotic nervous system',
    icon: '🤖',
  },
  {
    id: 'simulation',
    label: 'Simulation',
    description: 'Build digital twins with Gazebo & Unity',
    icon: '🎮',
  },
  {
    id: 'isaac',
    label: 'NVIDIA Isaac',
    description: 'Train AI brains for robots',
    icon: '🧠',
  },
  {
    id: 'vla',
    label: 'VLA Models',
    description: 'Vision-Language-Action AI',
    icon: '👁️',
  },
  {
    id: 'capstone',
    label: 'Capstone Project',
    description: 'Build a complete humanoid robot application',
    icon: '🏆',
  },
];

export function GoalsSelector({
  selectedGoals,
  onChange,
}: GoalsSelectorProps): JSX.Element {
  const toggleGoal = (goalId: string) => {
    if (selectedGoals.includes(goalId)) {
      onChange(selectedGoals.filter((g) => g !== goalId));
    } else {
      onChange([...selectedGoals, goalId]);
    }
  };

  return (
    <div className={styles.goalsSelector}>
      <h3 className={styles.skillLabel}>What do you want to learn?</h3>
      <p className={styles.goalsHint}>Select all that apply</p>
      <div className={styles.goalsGrid}>
        {availableGoals.map((goal) => (
          <button
            key={goal.id}
            type="button"
            className={`${styles.goalCard} ${
              selectedGoals.includes(goal.id) ? styles.goalSelected : ''
            }`}
            onClick={() => toggleGoal(goal.id)}
          >
            <span className={styles.goalIcon}>{goal.icon}</span>
            <span className={styles.goalLabel}>{goal.label}</span>
            <span className={styles.goalDescription}>{goal.description}</span>
            {selectedGoals.includes(goal.id) && (
              <span className={styles.checkmark}>✓</span>
            )}
          </button>
        ))}
      </div>
    </div>
  );
}

export default GoalsSelector;
