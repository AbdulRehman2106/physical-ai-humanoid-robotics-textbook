import { Router, Request, Response, NextFunction } from 'express';
import { ChatService } from '../../services/ChatService';
import { validateBody, validateParams, schemas } from '../middleware/validation';
import { logger } from '../../utils/logger';

const router = Router();

// ChatService instance will be injected
let chatService: ChatService;

/**
 * Initialize chat routes with ChatService
 */
export function initChatRoutes(service: ChatService): void {
  chatService = service;
}

/**
 * POST /api/chat/conversations
 * Create new conversation
 */
router.post('/', async (_req: Request, res: Response, next: NextFunction) => {
  try {
    const conversation = await chatService.createConversation();

    res.status(201).json({
      id: conversation.id,
      createdAt: conversation.createdAt.toISOString(),
      updatedAt: conversation.createdAt.toISOString(),
      messages: [],
      status: conversation.status,
    });
  } catch (error) {
    next(error);
  }
});

/**
 * POST /api/chat/conversations/:conversationId/messages
 * Send message to conversation
 */
router.post(
  '/:conversationId/messages',
  validateParams(schemas.conversationId),
  validateBody(schemas.sendMessage),
  async (req: Request, res: Response, next: NextFunction) => {
    try {
      const { conversationId } = req.params;
      const { content } = req.body;

      logger.debug('Received message request', {
        conversationId,
        contentLength: content.length,
      });

      const message = await chatService.sendMessage(conversationId, content);

      res.status(200).json({
        id: message.id,
        conversationId: message.conversationId,
        role: message.role,
        content: message.content,
        timestamp: message.timestamp.toISOString(),
        metadata: message.metadata,
      });
    } catch (error) {
      next(error);
    }
  }
);

/**
 * GET /api/chat/conversations/:conversationId
 * Get conversation history
 */
router.get(
  '/:conversationId',
  validateParams(schemas.conversationId),
  async (req: Request, res: Response, next: NextFunction) => {
    try {
      const { conversationId } = req.params;

      const conversation = await chatService.getConversation(conversationId);

      res.status(200).json({
        id: conversation.id,
        createdAt: conversation.createdAt.toISOString(),
        updatedAt: conversation.updatedAt.toISOString(),
        messages: conversation.messages.map(msg => ({
          ...msg,
          timestamp: msg.timestamp.toISOString(),
        })),
        context: conversation.context,
        status: conversation.status,
      });
    } catch (error) {
      next(error);
    }
  }
);

/**
 * DELETE /api/chat/conversations/:conversationId
 * End conversation
 */
router.delete(
  '/:conversationId',
  validateParams(schemas.conversationId),
  async (req: Request, res: Response, next: NextFunction) => {
    try {
      const { conversationId } = req.params;

      await chatService.endConversation(conversationId);

      res.status(204).send();
    } catch (error) {
      next(error);
    }
  }
);

export default router;
