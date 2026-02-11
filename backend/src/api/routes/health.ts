import { Router, Request, Response } from 'express';
import { CohereAdapter } from '../../agent/providers/CohereAdapter';
import { logger } from '../../utils/logger';

const router = Router();

// Initialize Cohere provider for health check
let cohereProvider: CohereAdapter | null = null;

/**
 * Initialize health check with Cohere provider
 */
export function initHealthCheck(provider: CohereAdapter): void {
  cohereProvider = provider;
}

/**
 * GET /api/health
 * Health check endpoint
 */
router.get('/', async (_req: Request, res: Response) => {
  const startTime = Date.now();

  try {
    // Check Cohere provider health
    const cohereHealthy = cohereProvider ? await cohereProvider.healthCheck() : false;

    const status = cohereHealthy ? 'healthy' : 'degraded';
    const statusCode = cohereHealthy ? 200 : 503;

    const response = {
      status,
      timestamp: new Date().toISOString(),
      services: {
        cohere: cohereHealthy ? 'available' : 'unavailable',
      },
      latency: Date.now() - startTime,
    };

    logger.info('Health check completed', response);

    res.status(statusCode).json(response);
  } catch (error) {
    logger.error('Health check failed', { error });

    res.status(503).json({
      status: 'unhealthy',
      timestamp: new Date().toISOString(),
      services: {
        cohere: 'error',
      },
      error: 'Health check failed',
    });
  }
});

export default router;
