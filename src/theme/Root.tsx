/**
 * Root Component Wrapper
 *
 * Wraps the entire app to include global components
 */

import React from 'react';
import {
  ReadingProgress,
  FloatingActionButton,
  KeyboardShortcutsPanel,
  FontSizeController,
  BookmarkManager,
  NotesPanel,
  LearningProgressTracker,
  QuickFeedback,
  MultiThemeSelector
} from '../components';

export default function Root({children}) {
  return (
    <>
      <ReadingProgress />
      <MultiThemeSelector />
      {children}
      <FloatingActionButton />
      <KeyboardShortcutsPanel />
      <FontSizeController />
      <BookmarkManager />
      <NotesPanel />
      <LearningProgressTracker />
      <QuickFeedback />
    </>
  );
}
