import React, { useEffect, useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import clsx from 'clsx';
import styles from './PersonalizedIntroduction.module.css';
import API_CONFIG from '../../config/api';

interface PersonalizedIntroductionProps {
  chapterTitle: string;
  chapterId?: string;
}

const PersonalizedIntroduction: React.FC<PersonalizedIntroductionProps> = ({
  chapterTitle,
  chapterId = 'unknown'
}) => {
  const auth = useAuth();
  const user = auth?.user || null;
  const isAuthenticated = auth?.isAuthenticated || false;
  const [personalizationData, setPersonalizationData] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchPersonalization = async () => {
      if (!isAuthenticated || !user) {
        setLoading(false);
        return;
      }

      try {
        const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/personalize/${chapterId}`, {
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('access_token')}`
          }
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({ detail: 'Failed to fetch personalization data' }));
          throw new Error(errorData.detail || 'Failed to fetch personalization data');
        }

        const data = await response.json();
        setPersonalizationData(data);
      } catch (err) {
        if (err instanceof Error) {
          setError(err.message);
        } else if (typeof err === 'string') {
          setError(err);
        } else if (typeof err === 'object' && err !== null && 'message' in err) {
          setError((err as any).message || 'An error occurred');
        } else {
          setError('An error occurred');
        }
      } finally {
        setLoading(false);
      }
    };

    fetchPersonalization();
  }, [isAuthenticated, user, chapterId]);

  if (!isAuthenticated || !user) {
    return (
      <div className={clsx(styles.personalizedIntro, styles.notLoggedIn)}>
        <div className="alert alert--info">
          <p>Sign in to see personalized content recommendations based on your background and goals.</p>
          <a href="/signin" className={styles.signInLink}>Sign In</a>
        </div>
      </div>
    );
  }

  if (loading) {
    return (
      <div className={styles.personalizedIntro}>
        <div className="alert alert--info">
          <p>Preparing personalized content for you...</p>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.personalizedIntro}>
        <div className="alert alert--danger">
          <p>Error loading personalized content: {error}</p>
        </div>
      </div>
    );
  }

  // If no personalization data, show a generic message
  if (!personalizationData) {
    return (
      <div className={styles.personalizedIntro}>
        <div className="alert alert--info">
          <p>Welcome! This chapter on {chapterTitle} will be tailored to your experience level.</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.personalizedIntro}>
      <div className="alert alert--success">
        <h3>Personalized for You</h3>
        <p>
          Based on your background ({user.software_background_level} in software and {user.hardware_background_level} in hardware),
          this chapter on <strong>{chapterTitle}</strong> will be adapted to your experience level.
        </p>

        {personalizationData.difficulty_level && (
          <div className={styles.difficultyInfo}>
            <strong>Difficulty Level:</strong> {personalizationData.difficulty_level}
          </div>
        )}

        {personalizationData.learning_objectives && personalizationData.learning_objectives.length > 0 && (
          <div className={styles.learningObjectives}>
            <strong>Learning Objectives:</strong>
            <ul>
              {personalizationData.learning_objectives.slice(0, 3).map((objective: string, index: number) => (
                <li key={index}>{objective}</li>
              ))}
            </ul>
          </div>
        )}

        {personalizationData.recommended_prerequisites && personalizationData.recommended_prerequisites.length > 0 && (
          <div className={styles.prerequisites}>
            <strong>Recommended Prerequisites:</strong>
            <ul>
              {personalizationData.recommended_prerequisites.slice(0, 2).map((prereq: string, index: number) => (
                <li key={index}>{prereq}</li>
              ))}
            </ul>
          </div>
        )}

        {personalizationData.practice_exercises && personalizationData.practice_exercises.length > 0 && (
          <div className={styles.exercises}>
            <strong>Suggested Exercises:</strong>
            <ul>
              {personalizationData.practice_exercises.slice(0, 2).map((exercise: any, index: number) => (
                <li key={index}>{exercise.description} ({exercise.type})</li>
              ))}
            </ul>
          </div>
        )}

        {personalizationData.additional_resources && personalizationData.additional_resources.length > 0 && (
          <div className={styles.resources}>
            <strong>Additional Resources:</strong>
            <ul>
              {personalizationData.additional_resources.slice(0, 3).map((resource: string, index: number) => (
                <li key={index}>{resource}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default PersonalizedIntroduction;