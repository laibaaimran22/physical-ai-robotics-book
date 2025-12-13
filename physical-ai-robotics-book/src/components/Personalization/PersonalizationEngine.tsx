import React, { ReactNode } from 'react';
import { useAuth } from '../../hooks/useAuth';

interface PersonalizedContentProps {
  children: ReactNode;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  contentType?: 'text' | 'code' | 'example' | 'explanation';
  fallbackContent?: ReactNode;
}

const PersonalizationEngine: React.FC<PersonalizedContentProps> = ({
  children,
  difficulty,
  contentType = 'text',
  fallbackContent
}) => {
  const auth = useAuth();
  const user = auth?.user || null;
  const isAuthenticated = auth?.isAuthenticated || false;

  // If user is not authenticated → show default content
  if (!isAuthenticated || !user) {
    return <>{children}</>;
  }

  // Determine if the content should be shown based on user background
  const shouldShowContent = () => {
    if (!difficulty) return true;

    const getLevelValue = (level: string | undefined) => {
      if (!level) return 0;
      switch (level.toLowerCase()) {
        case 'beginner': return 1;
        case 'intermediate': return 2;
        case 'advanced': return 3;
        default: return 0;
      }
    };

    let userLevel = 0;

    if (contentType === 'code' || contentType === 'example') {
      userLevel = getLevelValue(user.software_background_level);
    } else if (contentType === 'explanation') {
      userLevel = Math.max(
        getLevelValue(user.software_background_level),
        getLevelValue(user.hardware_background_level)
      );
    } else {
      userLevel = Math.max(
        getLevelValue(user.software_background_level),
        getLevelValue(user.hardware_background_level)
      );
    }

    const requiredLevel = getLevelValue(difficulty);
    return userLevel >= requiredLevel;
  };

  if (!shouldShowContent()) {
    return (
      <>
        {fallbackContent || (
          <p>
            This content is more advanced than your current level. 
            Please review foundational materials first.
          </p>
        )}
      </>
    );
  }

  return <>{children}</>;
};

// ==========================
// PERSONALIZED INTRO SECTION
// ==========================

interface PersonalizedIntroductionProps {
  chapterTitle: string;
  children: ReactNode;
}

export const PersonalizedIntroduction: React.FC<PersonalizedIntroductionProps> = ({
  chapterTitle,
  children
}) => {
  const auth = useAuth();
  const user = auth?.user || null;
  const isAuthenticated = auth?.isAuthenticated || false;

  // If not logged in → show plain content
  if (!isAuthenticated || !user) {
    return <div className="personalized-intro">{children}</div>;
  }

  const getPersonalizedGreeting = () => {
    if (user.software_background_level && user.hardware_background_level) {
      return `Based on your background (${user.software_background_level} in software and ${user.hardware_background_level} in hardware), this chapter on ${chapterTitle} will match your skill level.`;
    } 
    if (user.software_background_level) {
      return `Based on your ${user.software_background_level} software background, this chapter on ${chapterTitle} will match your skill level.`;
    }
    if (user.hardware_background_level) {
      return `Based on your ${user.hardware_background_level} hardware background, this chapter on ${chapterTitle} will match your skill level.`;
    }

    return `Welcome! This chapter on ${chapterTitle} will adapt to your profile.`;
  };

  return (
    <div className="personalized-intro">
      <div className="alert alert--info" style={{ marginBottom: '1.5rem' }}>
        <p><strong>Personalized for you:</strong> {getPersonalizedGreeting()}</p>
      </div>
      {children}
    </div>
  );
};

export default PersonalizationEngine;
