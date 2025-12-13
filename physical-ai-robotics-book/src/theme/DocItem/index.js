import React from 'react';
import OriginalDocItem from '@theme-original/DocItem';
import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

// Custom DocItem that wraps content with translation functionality
export default function DocItem(props) {
  return (
    <ChapterTranslator>
      <OriginalDocItem {...props} />
    </ChapterTranslator>
  );
}