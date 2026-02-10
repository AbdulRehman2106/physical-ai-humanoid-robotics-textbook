/**
 * Custom error classes for the AI Chatbot Assistant
 */

/**
 * Base error class for agent-related errors
 */
export class AgentError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'AgentError';
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error thrown by provider operations (Cohere API, etc.)
 */
export class ProviderError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'ProviderError';
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error thrown by tool operations
 */
export class ToolError extends Error {
  constructor(
    message: string,
    public toolName: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ToolError';
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error thrown for validation failures
 */
export class ValidationError extends Error {
  constructor(
    message: string,
    public field?: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ValidationError';
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error thrown when a resource is not found
 */
export class NotFoundError extends Error {
  constructor(
    message: string,
    public resourceType: string,
    public resourceId?: string
  ) {
    super(message);
    this.name = 'NotFoundError';
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error thrown for rate limiting
 */
export class RateLimitError extends Error {
  constructor(
    message: string,
    public retryAfter?: number
  ) {
    super(message);
    this.name = 'RateLimitError';
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Error thrown for timeout scenarios
 */
export class TimeoutError extends Error {
  constructor(
    message: string,
    public timeoutMs: number
  ) {
    super(message);
    this.name = 'TimeoutError';
    Error.captureStackTrace(this, this.constructor);
  }
}
