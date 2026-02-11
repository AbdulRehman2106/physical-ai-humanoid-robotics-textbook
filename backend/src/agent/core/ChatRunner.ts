import { IRunner } from './Runner';
import { IAgent } from './Agent';
import { IMemoryManager } from '../memory/MemoryManager';
import { IContext, AgentResponse } from './types';
import { PromptOrchestrator } from '../prompts/PromptOrchestrator';
import { logger } from '../../utils/logger';
import { NotFoundError } from '../../utils/errors';

/**
 * Chat runner implementation
 *
 * Orchestrates agent execution and manages conversation flow.
 * Coordinates between Agent, Memory, and PromptOrchestrator.
 */
export class ChatRunner implements IRunner {
  private agent: IAgent;
  private memory: IMemoryManager;
  private orchestrator: PromptOrchestrator;

  constructor(agent: IAgent, memory: IMemoryManager) {
    this.agent = agent;
    this.memory = memory;
    this.orchestrator = new PromptOrchestrator();

    logger.info('ChatRunner initialized');
  }

  /**
   * Execute agent with user input
   */
  async run(conversationId: string, userInput: string): Promise<AgentResponse> {
    try {
      logger.debug('Runner executing', {
        conversationId,
        inputLength: userInput.length,
      });

      // Get conversation context
      const context = this.memory.getContext(conversationId);
      if (!context) {
        throw new NotFoundError(
          `Conversation ${conversationId} not found`,
          'Conversation',
          conversationId
        );
      }

      // Assemble messages with system prompt
      const messages = this.orchestrator.assembleMessages(
        userInput,
        context.messages
      );

      // Update context with assembled messages
      const updatedContext: IContext = {
        ...context,
        messages,
      };

      // Run agent
      const response = await this.agent.run(userInput, updatedContext);

      // Store user message
      this.memory.addMessage(conversationId, {
        role: 'user',
        content: userInput,
      });

      // Store assistant response
      this.memory.addMessage(conversationId, {
        role: 'assistant',
        content: response.content,
      });

      // Update context
      this.memory.updateContext(conversationId, {
        lastUpdated: new Date(),
      });

      logger.info('Runner execution completed', {
        conversationId,
        responseLength: response.content.length,
        format: response.format,
      });

      return response;
    } catch (error) {
      logger.error('Runner execution failed', {
        error,
        conversationId,
      });
      throw error;
    }
  }

  /**
   * Create new conversation
   */
  createConversation(): string {
    const conversationId = this.memory.createConversation();
    logger.info('Conversation created via runner', { conversationId });
    return conversationId;
  }

  /**
   * End conversation
   */
  endConversation(conversationId: string): void {
    this.memory.endConversation(conversationId);
    logger.info('Conversation ended via runner', { conversationId });
  }

  /**
   * Get conversation context
   */
  getContext(conversationId: string): IContext | undefined {
    return this.memory.getContext(conversationId);
  }
}
