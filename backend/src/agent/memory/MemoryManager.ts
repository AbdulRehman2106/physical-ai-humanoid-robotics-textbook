import { ChatMessage, IContext } from '../core/types';

/**
 * Memory manager interface
 *
 * Manages conversation state and context.
 */
export interface IMemoryManager {
  /**
   * Store message in conversation
   *
   * @param conversationId - Conversation identifier
   * @param message - Message to store
   */
  addMessage(conversationId: string, message: ChatMessage): void;

  /**
   * Get conversation messages
   *
   * @param conversationId - Conversation identifier
   * @returns Array of messages
   */
  getMessages(conversationId: string): ChatMessage[];

  /**
   * Get conversation context
   *
   * @param conversationId - Conversation identifier
   * @returns Conversation context
   */
  getContext(conversationId: string): IContext | undefined;

  /**
   * Update conversation context
   *
   * @param conversationId - Conversation identifier
   * @param context - Updated context
   */
  updateContext(conversationId: string, context: Partial<IContext>): void;

  /**
   * Clear conversation
   *
   * @param conversationId - Conversation identifier
   */
  clearConversation(conversationId: string): void;

  /**
   * Prune old messages from conversation
   *
   * @param conversationId - Conversation identifier
   * @param keepCount - Number of recent messages to keep
   */
  pruneMessages(conversationId: string, keepCount: number): void;
}
