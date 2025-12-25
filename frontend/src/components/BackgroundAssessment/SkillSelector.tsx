/**
 * Skill level selector component for background assessment.
 */

import React from 'react';
import styles from './BackgroundAssessment.module.css';

interface SkillOption {
  value: string;
  label: string;
  description: string;
}

interface SkillSelectorProps {
  label: string;
  value: string | undefined;
  options: SkillOption[];
  onChange: (value: string) => void;
}

export function SkillSelector({
  label,
  value,
  options,
  onChange,
}: SkillSelectorProps): JSX.Element {
  return (
    <div className={styles.skillSelector}>
      <h3 className={styles.skillLabel}>{label}</h3>
      <div className={styles.optionGrid}>
        {options.map((option) => (
          <button
            key={option.value}
            type="button"
            className={`${styles.optionCard} ${
              value === option.value ? styles.optionSelected : ''
            }`}
            onClick={() => onChange(option.value)}
          >
            <span className={styles.optionLabel}>{option.label}</span>
            <span className={styles.optionDescription}>{option.description}</span>
          </button>
        ))}
      </div>
    </div>
  );
}

// Predefined skill options
export const pythonLevelOptions: SkillOption[] = [
  {
    value: 'beginner',
    label: 'Beginner',
    description: 'New to Python or basic syntax knowledge',
  },
  {
    value: 'intermediate',
    label: 'Intermediate',
    description: 'Comfortable with OOP, functions, and modules',
  },
  {
    value: 'advanced',
    label: 'Advanced',
    description: 'Expert with async, decorators, and best practices',
  },
];

export const rosExperienceOptions: SkillOption[] = [
  {
    value: 'none',
    label: 'No Experience',
    description: 'Never used ROS or ROS 2 before',
  },
  {
    value: 'basic',
    label: 'Basic',
    description: 'Completed tutorials or simple projects',
  },
  {
    value: 'experienced',
    label: 'Experienced',
    description: 'Built real projects with ROS/ROS 2',
  },
];

export const hardwareAccessOptions: SkillOption[] = [
  {
    value: 'simulation_only',
    label: 'Simulation Only',
    description: 'No physical hardware, using simulators',
  },
  {
    value: 'jetson',
    label: 'Jetson/SBC',
    description: 'Have Jetson Nano/Xavier or Raspberry Pi',
  },
  {
    value: 'full_kit',
    label: 'Full Kit',
    description: 'Complete robot kit with sensors and actuators',
  },
];

export default SkillSelector;
