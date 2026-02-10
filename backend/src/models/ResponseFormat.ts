/**
 * Response format types and detection
 */

export type ResponseFormat = 'plain' | 'structured' | 'code' | 'table';

export interface FormattedResponse {
  content: string;
  format: ResponseFormat;
  metadata: {
    hasCodeBlocks: boolean;
    hasLists: boolean;
    hasHeaders: boolean;
    hasTables: boolean;
  };
}

/**
 * Detect response format from content
 */
export function detectFormat(content: string): ResponseFormat {
  const hasCodeBlocks = /```[\s\S]*?```/.test(content);
  const hasHeaders = /^#{1,6}\s/m.test(content);
  const hasLists = /^[\*\-\+]\s|^\d+\.\s/m.test(content);
  const hasTables = /\|.*\|/.test(content);

  if (hasTables) return 'table';
  if (hasCodeBlocks) return 'code';
  if (hasHeaders || hasLists) return 'structured';
  return 'plain';
}

/**
 * Detect format metadata
 */
export function detectFormatMetadata(content: string): FormattedResponse['metadata'] {
  return {
    hasCodeBlocks: /```[\s\S]*?```/.test(content),
    hasLists: /^[\*\-\+]\s|^\d+\.\s/m.test(content),
    hasHeaders: /^#{1,6}\s/m.test(content),
    hasTables: /\|.*\|/.test(content),
  };
}

/**
 * Create formatted response
 */
export function createFormattedResponse(content: string): FormattedResponse {
  return {
    content,
    format: detectFormat(content),
    metadata: detectFormatMetadata(content),
  };
}
