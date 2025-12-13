import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import clsx from 'clsx';
import API_CONFIG from '../../config/api';

interface PersonalizationControlProps {
  chapterId: string;
  chapterTitle: string;
  className?: string;
}

const PersonalizationControl: React.FC<PersonalizationControlProps> = ({
  chapterId,
  chapterTitle,
  className
}) => {
  const auth = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const [preferences, setPreferences] = useState({
    difficulty_level: 'auto',
    content_focus: 'balanced',
    learning_style: 'comprehensive',
    pace: 'moderate'
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  // Load saved preferences if available
  useEffect(() => {
    const loadPreferences = async () => {
      if (!auth?.isAuthenticated || !auth.user) return;

      try {
        const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/personalize/${chapterId}/preferences`, {
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('access_token') || ''}`,
            'Content-Type': 'application/json'
          }
        });

        if (response.ok) {
          const data = await response.json();
          if (data.preferences) {
            setPreferences(data.preferences);
          }
        }
      } catch (err) {
        console.error('Error loading preferences:', err);
      }
    };

    loadPreferences();
  }, [auth?.isAuthenticated, auth?.user, chapterId]);

  const handleSavePreferences = async () => {
    if (!auth?.isAuthenticated || !auth.user) {
      setError('Please sign in to customize chapter content');
      return;
    }

    setLoading(true);
    setError(null);
    setSuccess(false);

    try {
      const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/auth/personalize/${chapterId}/preferences`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('access_token') || ''}`,
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_title: chapterTitle,
          preferences
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: 'Failed to save preferences' }));
        throw new Error(errorData.detail || 'Failed to save preferences');
      }

      setSuccess(true);
      setTimeout(() => {
        setSuccess(false);
        setIsOpen(false);
      }, 2000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred while saving preferences');
    } finally {
      setLoading(false);
    }
  };

  const handlePreferenceChange = (key: string, value: string) => {
    setPreferences(prev => ({
      ...prev,
      [key]: value
    }));
  };

  if (!auth?.isAuthenticated) {
    return (
      <div className={clsx('alert alert--info', className)}>
        <p>Sign in to personalize this chapter's content to match your learning style and background.</p>
        <a href="/signin" className="button button--primary button--sm">Sign In</a>
      </div>
    );
  }

  return (
    <div className={className}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className={clsx(
          'button',
          'button--secondary',
          'button--sm',
          { 'button--active': isOpen }
        )}
        style={{
          marginBottom: '1rem',
          display: 'flex',
          alignItems: 'center',
          gap: '0.5rem'
        }}
      >
        <span>🎨</span>
        {isOpen ? 'Hide Personalization' : 'Personalize Content'}
      </button>

      {isOpen && (
        <div className="card" style={{ marginTop: '1rem' }}>
          <div className="card__header">
            <h3>Customize Chapter Experience</h3>
            <p>Adjust content based on your background and preferences for <strong>{chapterTitle}</strong></p>
          </div>

          <div className="card__body">
            {error && (
              <div className="alert alert--danger" style={{ marginBottom: '1rem' }}>
                {error}
              </div>
            )}

            {success && (
              <div className="alert alert--success" style={{ marginBottom: '1rem' }}>
                Preferences saved successfully!
              </div>
            )}

            <div style={{ marginBottom: '1rem' }}>
              <label htmlFor="difficulty_level" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
                Content Difficulty:
              </label>
              <select
                id="difficulty_level"
                value={preferences.difficulty_level}
                onChange={(e) => handlePreferenceChange('difficulty_level', e.target.value)}
                className="form-control"
                style={{ width: '100%', padding: '0.5rem' }}
              >
                <option value="auto">Auto (based on your profile)</option>
                <option value="beginner">Beginner Friendly</option>
                <option value="intermediate">Intermediate Level</option>
                <option value="advanced">Advanced Content</option>
              </select>
            </div>

            <div style={{ marginBottom: '1rem' }}>
              <label htmlFor="content_focus" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
                Content Focus:
              </label>
              <select
                id="content_focus"
                value={preferences.content_focus}
                onChange={(e) => handlePreferenceChange('content_focus', e.target.value)}
                className="form-control"
                style={{ width: '100%', padding: '0.5rem' }}
              >
                <option value="balanced">Balanced (Theory + Practice)</option>
                <option value="theory">Theory Focused</option>
                <option value="practice">Practice Focused</option>
                <option value="examples">Example Heavy</option>
              </select>
            </div>

            <div style={{ marginBottom: '1rem' }}>
              <label htmlFor="learning_style" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
                Learning Style:
              </label>
              <select
                id="learning_style"
                value={preferences.learning_style}
                onChange={(e) => handlePreferenceChange('learning_style', e.target.value)}
                className="form-control"
                style={{ width: '100%', padding: '0.5rem' }}
              >
                <option value="comprehensive">Comprehensive (Detailed explanations)</option>
                <option value="concise">Concise (Key points only)</option>
                <option value="visual">Visual (Diagrams and illustrations)</option>
                <option value="hands-on">Hands-on (Code examples)</option>
              </select>
            </div>

            <div style={{ marginBottom: '1rem' }}>
              <label htmlFor="pace" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
                Learning Pace:
              </label>
              <select
                id="pace"
                value={preferences.pace}
                onChange={(e) => handlePreferenceChange('pace', e.target.value)}
                className="form-control"
                style={{ width: '100%', padding: '0.5rem' }}
              >
                <option value="slow">Slow (Take your time)</option>
                <option value="moderate">Moderate (Standard pace)</option>
                <option value="fast">Fast (Quick coverage)</option>
              </select>
            </div>

            <div style={{ display: 'flex', gap: '0.5rem', marginTop: '1rem' }}>
              <button
                onClick={handleSavePreferences}
                disabled={loading}
                className="button button--primary"
                style={{ flex: 1 }}
              >
                {loading ? 'Saving...' : 'Save Preferences'}
              </button>
              <button
                onClick={() => setIsOpen(false)}
                className="button button--secondary"
                style={{ flex: 1 }}
              >
                Cancel
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizationControl;