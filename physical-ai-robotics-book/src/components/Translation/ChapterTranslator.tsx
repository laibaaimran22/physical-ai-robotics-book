import React, { useState } from 'react';
import styles from './ChapterTranslator.module.css';

interface ChapterTranslatorProps {
  children: React.ReactNode;
}

export default function ChapterTranslator({ children }: ChapterTranslatorProps) {
  const [translated, setTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Recursively extract text from React children
  const extractText = (node: React.ReactNode): string => {
    if (typeof node === 'string') {
      return node;
    }

    if (typeof node === 'number') {
      return String(node);
    }

    if (node === null || node === undefined) {
      return '';
    }

    if (Array.isArray(node)) {
      return node.map(extractText).join(' ');
    }

    if (React.isValidElement(node)) {
      const props = node.props as { children?: React.ReactNode };
      if (props && props.children) {
        return extractText(props.children);
      }
    }

    return '';
  };

  // Translation using backend service as primary with MyMemory as fallback
  async function translateToUrdu(text: string): Promise<string> {
    // First try the backend translation API
    try {
      // Import API_CONFIG dynamically to avoid circular dependencies
      const API_CONFIG = (await import('../../config/api')).default;

      const response = await fetch(`${API_CONFIG.BACKEND_URL}/api/v1/translation`, {
        method: 'POST',
        credentials: 'include', // Include cookies for authentication if needed
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text: text,
          targetLanguage: 'ur',
          sourceLanguage: 'en'
        })
      });

      if (!response.ok) {
        throw new Error('Backend translation API request failed');
      }

      const data = await response.json();
      return data.translated_text || data.translatedText;
    } catch (backendErr) {
      console.error('Backend translation failed, falling back to MyMemory:', backendErr);

      // Fallback to MyMemory API
      const chunkSize = 500;
      const chunks: string[] = [];

      for (let i = 0; i < text.length; i += chunkSize) {
        chunks.push(text.substring(i, i + chunkSize));
      }

      try {
        const translatedChunks = await Promise.all(
          chunks.map(async (chunk) => {
            const response = await fetch(
              `https://api.mymemory.translated.net/get?q=${encodeURIComponent(
                chunk
              )}&langpair=en|ur`
            );

            if (!response.ok) {
              throw new Error('MyMemory translation API request failed');
            }

            const data = await response.json();

            if (data.responseStatus !== 200) {
              // Check if it's a rate limit error
              if (data.responseStatus === 429 || (data.responseDetails && data.responseDetails.includes("FREE TRANSLATIONS FOR TODAY"))) {
                throw new Error("Daily translation limit reached. Please try again tomorrow.");
              }
              throw new Error(data.responseDetails || 'Translation failed');
            }

            return data.responseData.translatedText;
          })
        );

        return translatedChunks.join(' ');
      } catch (mymemoryErr) {
        console.error('MyMemory translation also failed:', mymemoryErr);

        let errorMessage = 'Failed to translate content. Please try again.';
        if (mymemoryErr instanceof Error) {
          if (mymemoryErr.message.includes("FREE TRANSLATIONS FOR TODAY")) {
            errorMessage = "Daily translation limit reached. Please try again tomorrow.";
          } else {
            errorMessage = mymemoryErr.message;
          }
        }

        throw new Error(errorMessage);
      }
    }
  }

  const handleTranslate = async () => {
    if (translated) {
      // Switch back to English
      setTranslated(false);
      setTranslatedContent('');
      setError(null);
      return;
    }

    // Translate to Urdu
    setIsLoading(true);
    setError(null);

    try {
      const textToTranslate = extractText(children);

      if (!textToTranslate || textToTranslate.trim() === '') {
        throw new Error('No text content found to translate');
      }

      console.log('Text to translate:', (textToTranslate || '').substring(0, 100) + '...');

      const urduText = await translateToUrdu(textToTranslate);

      console.log('Translated text:', (urduText || '').substring(0, 100) + '...');

      setTranslatedContent(urduText);
      setTranslated(true);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed. Please try again.';
      setError(errorMessage);
      console.error('Translation error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Render content based on translation state
  const renderContent = () => {
    if (translated && translatedContent) {
      return (
        <div
          className={styles.urduContent}
          style={{
            direction: 'rtl',
            textAlign: 'right',
            fontFamily: 'Noto Nastaliq Urdu, "Jameel Noori Nastaleeq", "Urdu Typesetting", Arial, sans-serif',
            lineHeight: '2.2',
            fontSize: '1.15em',
            whiteSpace: 'pre-wrap'
          }}
        >
          {translatedContent}
        </div>
      );
    }

    return <div className={styles.englishContent}>{children}</div>;
  };

  return (
    <div className={styles.translatorWrapper}>
      <button
        className={styles.translateButton}
        onClick={handleTranslate}
        disabled={isLoading}
        aria-label={translated ? 'Switch to English' : 'Translate to Urdu'}
      >
        {isLoading ? (
          <span className={styles.buttonContent}>
            <span className={styles.loader}></span>
            Translating...
          </span>
        ) : translated ? (
          <span className={styles.buttonContent}>
            <span className={styles.icon}>🇬🇧</span>
            Show English
          </span>
        ) : (
          <span className={styles.buttonContent}>
            <span className={styles.icon}>🇵🇰</span>
            ترجمہ کریں (Urdu)
          </span>
        )}
      </button>

      {error && (
        <div className={styles.errorMessage}>
          <span>{error}</span>
          <button
            onClick={handleTranslate}
            className={styles.retryButton}
          >
            Retry
          </button>
        </div>
      )}

      <div className={styles.content}>
        {renderContent()}
      </div>
    </div>
  );
}