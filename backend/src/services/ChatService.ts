import { IRunner } from '../agent/core/Runner';
import { AgentResponse } from '../agent/core/types';
import { ValidationError, NotFoundError } from '../utils/errors';
import { logger } from '../utils/logger';
import { SafetyService } from './SafetyService';

/**
 * Chat service - Business logic for chat operations
 *
 * Handles conversation management and message processing
 * with validation and error handling.
 */
export class ChatService {
  private runner: IRunner;
  private safetyService: SafetyService;

  constructor(runner: IRunner) {
    this.runner = runner;
    this.safetyService = new SafetyService();
    logger.info('ChatService initialized');
  }

  /**
   * Create new conversation
   */
  async createConversation(): Promise<{ id: string; createdAt: Date; status: string }> {
    try {
      const conversationId = this.runner.createConversation();
      const context = this.runner.getContext(conversationId);

      if (!context) {
        throw new Error('Failed to create conversation');
      }

      logger.info('Conversation created', { conversationId });

      return {
        id: conversationId,
        createdAt: new Date(),
        status: 'active',
      };
    } catch (error) {
      logger.error('Failed to create conversation', { error });
      throw error;
    }
  }

  /**
   * Send message to conversation
   */
  async sendMessage(
    conversationId: string,
    content: string
  ): Promise<{
    id: string;
    conversationId: string;
    role: 'assistant';
    content: string;
    timestamp: Date;
    metadata: {
      model: string;
      tokens: number;
      latency: number;
    };
  }> {
    try {
      // Validate input
      this.validateMessageContent(content);

      // Safety pre-processing: Check for prohibited topics
      const refusalMessage = this.safetyService.preProcessInput(content);
      if (refusalMessage) {
        // Return refusal message immediately without calling LLM
        this.safetyService.logSafetyEvent('refusal', {
          conversationId,
          inputPreview: content.substring(0, 100),
        });

        const message = {
          id: this.generateMessageId(),
          conversationId,
          role: 'assistant' as const,
          content: refusalMessage,
          timestamp: new Date(),
          metadata: {
            model: 'safety-filter',
            tokens: 0,
            latency: 0,
          },
        };

        logger.info('Safety refusal returned', {
          conversationId,
          messageId: message.id,
        });

        return message;
      }

      // Check conversation exists
      const context = this.runner.getContext(conversationId);
      if (!context) {
        throw new NotFoundError(
          `Conversation ${conversationId} not found`,
          'Conversation',
          conversationId
        );
      }

      logger.debug('Sending message', {
        conversationId,
        contentLength: content.length,
      });

      // Process message through runner
      const response: AgentResponse = await this.runner.run(conversationId, content);

      // Safety post-processing: Validate response
      const validation = this.safetyService.postProcessResponse(response.content);
      if (!validation.isValid) {
        logger.warn('Response validation warnings', {
          conversationId,
          warnings: validation.warnings,
        });
      }

      if (validation.hasUncertainty) {
        this.safetyService.logSafetyEvent('uncertainty', {
          conversationId,
          responsePreview: response.content.substring(0, 100),
        });
      }

      // Build response message
      const message = {
        id: this.generateMessageId(),
        conversationId,
        role: 'assistant' as const,
        content: response.content,
        timestamp: new Date(),
        metadata: {
          model: response.metadata.model,
          tokens: response.metadata.tokens,
          latency: response.metadata.latency,
        },
      };

      logger.info('Message sent successfully', {
        conversationId,
        messageId: message.id,
        tokens: message.metadata.tokens,
        hasUncertainty: validation.hasUncertainty,
      });

      return message;
    } catch (error) {
      logger.error('Failed to send message', {
        error,
        conversationId,
      });
      throw error;
    }
  }

  /**
   * Get conversation history
   */
  async getConversation(conversationId: string): Promise<{
    id: string;
    createdAt: Date;
    updatedAt: Date;
    messages: Array<{
      id: string;
      role: 'user' | 'assistant';
      content: string;
      timestamp: Date;
      metadata?: any;
    }>;
    context: {
      summary: string;
      topics: string[];
      tokenCount: number;
    };
    status: string;
  }> {
    try {
      const context = this.runner.getContext(conversationId);
      if (!context) {
        throw new NotFoundError(
          `Conversation ${conversationId} not found`,
          'Conversation',
          conversationId
        );
      }

      // Get conversation from memory (assuming we have access)
      // For now, we'll construct from context
      const messages = context.messages.map((msg, index) => ({
        id: this.generateMessageId(),
        role: msg.role as 'user' | 'assistant',
        content: msg.content,
        timestamp: new Date(Date.now() - (context.messages.length - index) * 1000),
        metadata: undefined,
      }));

      logger.debug('Conversation retrieved', {
        conversationId,
        messageCount: messages.length,
      });

      return {
        id: conversationId,
        createdAt: new Date(Date.now() - messages.length * 1000), // Approximate
        updatedAt: context.lastUpdated || new Date(),
        messages,
        context: {
          summary: context.summary,
          topics: context.topics,
          tokenCount: context.tokenCount || 0,
        },
        status: 'active',
      };
    } catch (error) {
      logger.error('Failed to get conversation', {
        error,
        conversationId,
      });
      throw error;
    }
  }

  /**
   * End conversation
   */
  async endConversation(conversationId: string): Promise<void> {
    try {
      const context = this.runner.getContext(conversationId);
      if (!context) {
        throw new NotFoundError(
          `Conversation ${conversationId} not found`,
          'Conversation',
          conversationId
        );
      }

      this.runner.endConversation(conversationId);

      logger.info('Conversation ended', { conversationId });
    } catch (error) {
      logger.error('Failed to end conversation', {
        error,
        conversationId,
      });
      throw error;
    }
  }

  /**
   * Validate message content
   */
  private validateMessageContent(content: string): void {
    if (!content || content.trim().length === 0) {
      throw new ValidationError('Message content cannot be empty');
    }

    if (content.length > 10000) {
      throw new ValidationError(
        'Message content cannot exceed 10,000 characters',
        'content'
      );
    }
  }

  /**
   * Generate message ID
   */
  private generateMessageId(): string {
    return `msg_${Date.now()}_${Math.random().toString(36).substring(7)}`;
  }
}
