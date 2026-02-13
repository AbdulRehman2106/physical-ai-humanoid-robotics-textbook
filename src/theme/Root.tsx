/**
 * Root Component Wrapper
 *
 * Wraps the entire app to include global components
 */

import React from 'react';
import {
  ReadingProgress,
  MultiThemeSelector
} from '../components';
import ChatBot from '../components/ChatBot';

export default function Root({children}) {
  return (
    <>
      <ReadingProgress />
      <MultiThemeSelector />
      {children}
      <ChatBot />
    </>
  );
}
