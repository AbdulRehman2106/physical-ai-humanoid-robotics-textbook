import { Request, Response, NextFunction } from 'express';
import { logger } from '../../utils/logger';
import {
  AgentError,
  ProviderError,
  ToolError,
  ValidationError,
  NotFoundError,
  RateLimitError,
  TimeoutError,
} from '../../utils/errors';

/**
 * Error response structure
 */
interface ErrorResponse {
  error: string;
  message: string;
  code?: string;
  details?: any;
}

/**
 * Global error handling middleware
 *
 * Catches all errors and returns appropriate HTTP responses
 */
export function errorHandler(
  err: Error,
  req: Request,
  res: Response,
  _next: NextFunction
): void {
  // Log the error
  logger.error('Request error', {
    error: err.message,
    stack: err.stack,
    path: req.path,
    method: req.method,
  });

  // Handle specific error types
  if (err instanceof ValidationError) {
    res.status(400).json({
      error: 'VALIDATION_ERROR',
      message: err.message,
      code: '400',
      details: err.details,
    } as ErrorResponse);
    return;
  }

  if (err instanceof NotFoundError) {
    res.status(404).json({
      error: 'NOT_FOUND',
      message: err.message,
      code: '404',
      details: {
        resourceType: err.resourceType,
        resourceId: err.resourceId,
      },
    } as ErrorResponse);
    return;
  }

  if (err instanceof RateLimitError) {
    res.status(429).json({
      error: 'RATE_LIMIT_EXCEEDED',
      message: err.message,
      code: '429',
      details: {
        retryAfter: err.retryAfter,
      },
    } as ErrorResponse);
    return;
  }

  if (err instanceof TimeoutError) {
    res.status(504).json({
      error: 'TIMEOUT',
      message: err.message,
      code: '504',
      details: {
        timeoutMs: err.timeoutMs,
      },
    } as ErrorResponse);
    return;
  }

  if (err instanceof ProviderError) {
    const statusCode = err.statusCode || 500;
    res.status(statusCode).json({
      error: err.code,
      message: getUserFriendlyMessage(err),
      code: statusCode.toString(),
      details: err.details,
    } as ErrorResponse);
    return;
  }

  if (err instanceof AgentError) {
    res.status(500).json({
      error: err.code,
      message: err.message,
      code: '500',
      details: err.details,
    } as ErrorResponse);
    return;
  }

  if (err instanceof ToolError) {
    res.status(500).json({
      error: 'TOOL_ERROR',
      message: err.message,
      code: '500',
      details: {
        toolName: err.toolName,
        ...err.details,
      },
    } as ErrorResponse);
    return;
  }

  // Default error response
  res.status(500).json({
    error: 'INTERNAL_SERVER_ERROR',
    message: 'An unexpected error occurred. Please try again later.',
    code: '500',
  } as ErrorResponse);
}

/**
 * Get user-friendly error message for provider errors
 */
function getUserFriendlyMessage(error: ProviderError): string {
  switch (error.code) {
    case 'INVALID_API_KEY':
      return 'Service configuration error. Please contact support.';
    case 'RATE_LIMIT_EXCEEDED':
      return 'Service is busy. Please wait a moment and try again.';
    case 'SERVICE_ERROR':
      return 'AI service temporarily unavailable. Please try again in a few minutes.';
    case 'FORBIDDEN':
      return 'Access denied. Please check your permissions.';
    default:
      return error.message || 'An error occurred while processing your request.';
  }
}

/**
 * 404 handler for undefined routes
 */
export function notFoundHandler(req: Request, res: Response): void {
  res.status(404).json({
    error: 'NOT_FOUND',
    message: `Route ${req.method} ${req.path} not found`,
    code: '404',
  } as ErrorResponse);
}
