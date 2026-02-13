export interface SelectionContext {
  text: string;
  chapterTitle?: string;
  sectionTitle?: string;
  url: string;
}

/**
 * Capture selected text with context (chapter, section, URL)
 */
export function captureTextSelection(): SelectionContext | null {
  const selection = window.getSelection();

  if (!selection || selection.isCollapsed || !selection.rangeCount) {
    return null;
  }

  const selectedText = selection.toString().trim();

  if (!selectedText || selectedText.length < 3) {
    return null;
  }

  // Get the range and common ancestor
  const range = selection.getRangeAt(0);
  const container = range.commonAncestorContainer;

  // Find the closest article or main content element
  let element = container.nodeType === Node.TEXT_NODE
    ? container.parentElement
    : container as Element;

  // Traverse up to find chapter and section titles
  let chapterTitle: string | undefined;
  let sectionTitle: string | undefined;

  while (element && element !== document.body) {
    // Look for h1 (chapter title)
    if (!chapterTitle) {
      const h1 = element.querySelector('h1');
      if (h1) {
        chapterTitle = h1.textContent?.trim();
      }
    }

    // Look for closest h2 or h3 (section title)
    if (!sectionTitle) {
      // Check if current element is a heading
      if (element.tagName === 'H2' || element.tagName === 'H3') {
        sectionTitle = element.textContent?.trim();
      } else {
        // Look for previous heading sibling
        let sibling = element.previousElementSibling;
        while (sibling) {
          if (sibling.tagName === 'H2' || sibling.tagName === 'H3') {
            sectionTitle = sibling.textContent?.trim();
            break;
          }
          sibling = sibling.previousElementSibling;
        }
      }
    }

    element = element.parentElement as Element;
  }

  return {
    text: selectedText,
    chapterTitle,
    sectionTitle,
    url: window.location.href,
  };
}

/**
 * Check if there is currently selected text
 */
export function hasTextSelection(): boolean {
  const selection = window.getSelection();
  return !!(selection && !selection.isCollapsed && selection.toString().trim().length > 3);
}

/**
 * Clear current text selection
 */
export function clearTextSelection(): void {
  const selection = window.getSelection();
  if (selection) {
    selection.removeAllRanges();
  }
}
