import { IContext, AgentResponse } from './types';

/**
 * Runner interface
 *
 * Orchestrates agent execution and manages conversation flow.
 */
export interface IRunner {
  /**
   * Execute agent with user input
   *
   * @param conversationId - Conversation identifier
   * @param userInput - User message
   * @returns Promise resolving to agent response
   */
  run(conversationId: string, userInput: string): Promise<AgentResponse>;

  /**
   * Create new conversation
   *
   * @returns New conversation ID
   */
  createConversation(): string;

  /**
   * End conversation
   *
   * @param conversationId - Conversation to end
   */
  endConversation(conversationId: string): void;

  /**
   * Get conversation context
   *
   * @param conversationId - Conversation identifier
   * @returns Conversation context
   */
  getContext(conversationId: string): IContext | undefined;
}
