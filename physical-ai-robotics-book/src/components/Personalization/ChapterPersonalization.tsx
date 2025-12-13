import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import PersonalizationControl from './PersonalizationControl';

interface ChapterPersonalizationProps {
  chapterId: string;
  chapterTitle: string;
  className?: string;
}

const ChapterPersonalization: React.FC<ChapterPersonalizationProps> = ({
  chapterId,
  chapterTitle,
  className = ''
}) => {
  const auth = useAuth();
  const [hasShownNotification, setHasShownNotification] = useState(false);

  // Auto-hide notification after 5 seconds
  useEffect(() => {
    if (auth?.isAuthenticated && !hasShownNotification) {
      setHasShownNotification(true);
      const timer = setTimeout(() => {
        setHasShownNotification(false);
      }, 5000);

      return () => clearTimeout(timer);
    }
  }, [auth?.isAuthenticated, hasShownNotification]);

  if (!auth?.isAuthenticated) {
    return null; // Don't show anything to non-authenticated users
  }

  // Generate a chapter ID if one isn't provided
  const normalizedChapterId = chapterId || chapterTitle.toLowerCase().replace(/\s+/g, '-').replace(/[^\w-]/g, '');

  return (
    <div className={className}>
      <PersonalizationControl
        chapterId={normalizedChapterId}
        chapterTitle={chapterTitle}
      />

      {/* Notification for signed-in users */}
      {!hasShownNotification && (
        <div className="alert alert--info" style={{ marginBottom: '1rem' }}>
          <p>💡 You can personalize this chapter's content by clicking the "Personalize Content" button above.</p>
          <p>Your preferences will be saved and applied to future visits.</p>
        </div>
      )}
    </div>
  );
};

export default ChapterPersonalization;