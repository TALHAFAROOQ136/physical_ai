/**
 * Background assessment wizard component.
 */

import React, { useState } from 'react';
import {
  SkillSelector,
  pythonLevelOptions,
  rosExperienceOptions,
  hardwareAccessOptions,
} from './SkillSelector';
import { GoalsSelector } from './GoalsSelector';
import { useAuth } from '../../context/AuthContext';
import { BackgroundAssessment as BackgroundData } from '../../services/auth';
import styles from './BackgroundAssessment.module.css';

interface BackgroundAssessmentProps {
  onComplete?: () => void;
  initialData?: BackgroundData;
}

type Step = 'python' | 'ros' | 'hardware' | 'goals' | 'complete';

const steps: Step[] = ['python', 'ros', 'hardware', 'goals', 'complete'];

export function BackgroundAssessment({
  onComplete,
  initialData,
}: BackgroundAssessmentProps): JSX.Element {
  const { updateBackground } = useAuth();
  const [currentStep, setCurrentStep] = useState<Step>('python');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form state
  const [pythonLevel, setPythonLevel] = useState<string | undefined>(
    initialData?.pythonLevel
  );
  const [rosExperience, setRosExperience] = useState<string | undefined>(
    initialData?.rosExperience
  );
  const [hardwareAccess, setHardwareAccess] = useState<string | undefined>(
    initialData?.hardwareAccess
  );
  const [learningGoals, setLearningGoals] = useState<string[]>(
    initialData?.learningGoals || []
  );

  const currentStepIndex = steps.indexOf(currentStep);
  const progress = ((currentStepIndex + 1) / steps.length) * 100;

  const canProceed = () => {
    switch (currentStep) {
      case 'python':
        return !!pythonLevel;
      case 'ros':
        return !!rosExperience;
      case 'hardware':
        return !!hardwareAccess;
      case 'goals':
        return learningGoals.length > 0;
      default:
        return true;
    }
  };

  const handleNext = async () => {
    const nextIndex = currentStepIndex + 1;
    if (nextIndex < steps.length) {
      setCurrentStep(steps[nextIndex]);
    }
  };

  const handleBack = () => {
    const prevIndex = currentStepIndex - 1;
    if (prevIndex >= 0) {
      setCurrentStep(steps[prevIndex]);
    }
  };

  const handleSubmit = async () => {
    setLoading(true);
    setError(null);

    try {
      await updateBackground({
        pythonLevel: pythonLevel as BackgroundData['pythonLevel'],
        rosExperience: rosExperience as BackgroundData['rosExperience'],
        hardwareAccess: hardwareAccess as BackgroundData['hardwareAccess'],
        learningGoals,
        completedAssessment: true,
      });
      setCurrentStep('complete');
      onComplete?.();
    } catch (err: unknown) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to save assessment';
      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  const renderStep = () => {
    switch (currentStep) {
      case 'python':
        return (
          <SkillSelector
            label="What's your Python experience level?"
            value={pythonLevel}
            options={pythonLevelOptions}
            onChange={setPythonLevel}
          />
        );
      case 'ros':
        return (
          <SkillSelector
            label="How much ROS/ROS 2 experience do you have?"
            value={rosExperience}
            options={rosExperienceOptions}
            onChange={setRosExperience}
          />
        );
      case 'hardware':
        return (
          <SkillSelector
            label="What hardware do you have access to?"
            value={hardwareAccess}
            options={hardwareAccessOptions}
            onChange={setHardwareAccess}
          />
        );
      case 'goals':
        return (
          <GoalsSelector
            selectedGoals={learningGoals}
            onChange={setLearningGoals}
          />
        );
      case 'complete':
        return (
          <div className={styles.completeStep}>
            <div className={styles.completeIcon}>🎉</div>
            <h2 className={styles.completeTitle}>You're all set!</h2>
            <p className={styles.completeText}>
              We've personalized your learning experience based on your background.
            </p>
            <button
              className={styles.startButton}
              onClick={() => (window.location.href = '/dashboard')}
            >
              Go to Dashboard
            </button>
          </div>
        );
    }
  };

  return (
    <div className={styles.wizard}>
      {currentStep !== 'complete' && (
        <>
          <div className={styles.header}>
            <h2 className={styles.title}>Tell us about yourself</h2>
            <p className={styles.subtitle}>
              This helps us personalize your learning experience
            </p>
          </div>

          <div className={styles.progressBar}>
            <div
              className={styles.progressFill}
              style={{ width: `${progress}%` }}
            />
          </div>

          <div className={styles.stepIndicator}>
            Step {currentStepIndex + 1} of {steps.length - 1}
          </div>
        </>
      )}

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.stepContent}>{renderStep()}</div>

      {currentStep !== 'complete' && (
        <div className={styles.navigation}>
          <button
            type="button"
            className={styles.backButton}
            onClick={handleBack}
            disabled={currentStepIndex === 0}
          >
            Back
          </button>

          {currentStep === 'goals' ? (
            <button
              type="button"
              className={styles.nextButton}
              onClick={handleSubmit}
              disabled={!canProceed() || loading}
            >
              {loading ? 'Saving...' : 'Complete Setup'}
            </button>
          ) : (
            <button
              type="button"
              className={styles.nextButton}
              onClick={handleNext}
              disabled={!canProceed()}
            >
              Next
            </button>
          )}
        </div>
      )}
    </div>
  );
}

export default BackgroundAssessment;
export { SkillSelector, GoalsSelector };
