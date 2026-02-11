import express, { Express } from 'express';
import cors from 'cors';
import helmet from 'helmet';
import { config, validateConfig } from '../utils/config';
import { logger } from '../utils/logger';
import { errorHandler, notFoundHandler } from './middleware/errorHandler';
import healthRouter, { initHealthCheck } from './routes/health';
import chatRouter, { initChatRoutes } from './routes/chat';
import { CohereAdapter } from '../agent/providers/CohereAdapter';
import { ChatAgent } from '../agent/core/ChatAgent';
import { ChatRunner } from '../agent/core/ChatRunner';
import { ConversationMemory } from '../agent/memory/ConversationMemory';
import { ChatService } from '../services/ChatService';

/**
 * Create and configure Express application
 */
export function createApp(): Express {
  const app = express();

  // Security middleware
  app.use(helmet());

  // CORS configuration
  app.use(cors({
    origin: config.server.nodeEnv === 'production'
      ? process.env.ALLOWED_ORIGINS?.split(',') || [
          'https://physical-ai-frontend-kappa.vercel.app',
          'https://physical-ai-humanoid-robotics-textb-nine-neon.vercel.app',
          'https://physical-ai-humanoid-robotics-textbook-igiy4hlmc.vercel.app'
        ]
      : ['http://localhost:3002', 'http://localhost:3001', 'http://localhost:5173', 'http://localhost:3000'],
    credentials: true,
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
    allowedHeaders: ['Content-Type', 'Authorization', 'X-Requested-With'],
    exposedHeaders: ['Content-Length', 'X-Request-Id'],
    maxAge: 86400, // 24 hours
  }));

  // Body parsing middleware
  app.use(express.json({ limit: '1mb' }));
  app.use(express.urlencoded({ extended: true, limit: '1mb' }));

  // Request logging middleware
  app.use((req, _res, next) => {
    logger.info('Incoming request', {
      method: req.method,
      path: req.path,
      ip: req.ip,
    });
    next();
  });

  // Initialize components
  logger.info('Initializing AI Chatbot components...');

  // 1. Initialize Cohere provider
  const cohereProvider = new CohereAdapter();
  initHealthCheck(cohereProvider);
  logger.info('✅ Cohere provider initialized');

  // 2. Initialize memory manager
  const memory = new ConversationMemory();
  logger.info('✅ Conversation memory initialized');

  // 3. Initialize agent
  const agent = new ChatAgent(cohereProvider);
  logger.info('✅ Chat agent initialized');

  // 4. Initialize runner
  const runner = new ChatRunner(agent, memory);
  logger.info('✅ Chat runner initialized');

  // 5. Initialize chat service
  const chatService = new ChatService(runner);
  initChatRoutes(chatService);
  logger.info('✅ Chat service initialized');

  // Routes
  app.use('/api/health', healthRouter);
  app.use('/api/chat/conversations', chatRouter);

  // 404 handler
  app.use(notFoundHandler);

  // Error handling middleware (must be last)
  app.use(errorHandler);

  logger.info('✅ All components initialized successfully');

  return app;
}

/**
 * Start the server
 */
export async function startServer(): Promise<void> {
  try {
    // Validate configuration
    validateConfig();
    logger.info('Configuration validated successfully');

    // Create app
    const app = createApp();

    // Start listening
    const port = config.server.port;
    app.listen(port, () => {
      logger.info('🚀 AI Chatbot Assistant API');
      logger.info(`📡 Server listening on http://localhost:${port}`);
      logger.info(`✅ Environment: ${config.server.nodeEnv}`);
      logger.info(`✅ Cohere model: ${config.cohere.model}`);
    });
  } catch (error) {
    logger.error('Failed to start server', { error });
    process.exit(1);
  }
}

// Start server if this file is run directly
if (require.main === module) {
  startServer();
}
