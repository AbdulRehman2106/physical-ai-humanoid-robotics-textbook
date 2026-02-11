import { Request, Response, NextFunction } from 'express';
import { z, ZodError, ZodSchema } from 'zod';
import { ValidationError } from '../../utils/errors';

/**
 * Validate request body against Zod schema
 */
export function validateBody(schema: ZodSchema) {
  return (req: Request, _res: Response, next: NextFunction): void => {
    try {
      req.body = schema.parse(req.body);
      next();
    } catch (error) {
      if (error instanceof ZodError) {
        next(new ValidationError(
          'Request validation failed',
          undefined,
          error.errors
        ));
      } else {
        next(error);
      }
    }
  };
}

/**
 * Validate request params against Zod schema
 */
export function validateParams(schema: ZodSchema) {
  return (req: Request, _res: Response, next: NextFunction): void => {
    try {
      req.params = schema.parse(req.params);
      next();
    } catch (error) {
      if (error instanceof ZodError) {
        next(new ValidationError(
          'Parameter validation failed',
          undefined,
          error.errors
        ));
      } else {
        next(error);
      }
    }
  };
}

/**
 * Validate request query against Zod schema
 */
export function validateQuery(schema: ZodSchema) {
  return (req: Request, _res: Response, next: NextFunction): void => {
    try {
      req.query = schema.parse(req.query);
      next();
    } catch (error) {
      if (error instanceof ZodError) {
        next(new ValidationError(
          'Query validation failed',
          undefined,
          error.errors
        ));
      } else {
        next(error);
      }
    }
  };
}

/**
 * Common validation schemas
 */
export const schemas = {
  uuid: z.string().uuid('Invalid UUID format'),

  conversationId: z.object({
    conversationId: z.string().uuid('Invalid conversation ID'),
  }),

  sendMessage: z.object({
    content: z.string()
      .min(1, 'Message content cannot be empty')
      .max(10000, 'Message content cannot exceed 10,000 characters'),
  }),
};
