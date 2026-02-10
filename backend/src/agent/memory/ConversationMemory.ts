import { v4 as uuidv4 } from 'uuid';
import { IMemoryManager } from './MemoryManager';
import { ChatMessage, IContext } from '../core/types';
import { Conversation, Context, createConversation, createContext } from '../../models/Conversation';
import { logger } from '../../utils/logger';
import { config } from '../../utils/config';

/**
 * In-memory conversation storage and management
 *
 * Manages conversation state, context, and message history
 * with session timeout and context pruning.
 */
export class ConversationMemory implements IMemoryManager {
  private conversations: Map<string, Conversation> = new Map();
  private cleanupInterval: NodeJS.Timeout | null = null;

  constructor() {
    // Start cleanup job for expired sessions
    this.startCleanupJob();
  }

  /**
   * Add message to conversation
   */
  addMessage(conversationId: string, message: ChatMessage): void {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) {
      throw new Error(`Conversation ${conversationId} not found`);
    }

    // Update conversation
    conversation.messages.push({
      id: uuidv4(),
      conversationId,
      role: message.role as 'user' | 'assistant',
      content: message.content,
      timestamp: new Date(),
    });
    conversation.updatedAt = new Date();

    // Update context
    this.updateContext(conversationId, {
      lastUpdated: new Date(),
    });

    // Auto-prune if token limit exceeded
    this.autoPruneIfNeeded(conversationId);

    logger.debug('Message added to conversation', {
      conversationId,
      role: message.role,
      messageCount: conversation.messages.length,
      tokenCount: conversation.context.tokenCount,
    });
  }

  /**
   * Get conversation messages
   */
  getMessages(conversationId: string): ChatMessage[] {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) {
      return [];
    }

    return conversation.messages.map(msg => ({
      role: msg.role as 'user' | 'assistant',
      content: msg.content,
    }));
  }

  /**
   * Get conversation context
   */
  getContext(conversationId: string): IContext | undefined {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) {
      return undefined;
    }

    return {
      conversationId: conversation.context.conversationId,
      messages: this.getMessages(conversationId),
      summary: conversation.context.summary,
      topics: conversation.context.topics,
      tokenCount: conversation.context.tokenCount,
      lastUpdated: conversation.context.lastUpdated,
    };
  }

  /**
   * Update conversation context
   */
  updateContext(conversationId: string, context: Partial<IContext>): void {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) {
      throw new Error(`Conversation ${conversationId} not found`);
    }

    // Update context fields
    if (context.summary !== undefined) {
      conversation.context.summary = context.summary;
    }
    if (context.topics !== undefined) {
      conversation.context.topics = context.topics;
    }
    if (context.tokenCount !== undefined) {
      conversation.context.tokenCount = context.tokenCount;
    }
    if (context.lastUpdated !== undefined) {
      conversation.context.lastUpdated = context.lastUpdated;
    }

    // Extract topics from recent messages if not provided
    if (!context.topics) {
      conversation.context.topics = this.extractTopics(conversation.messages.slice(-10));
    }

    // Estimate token count if not provided
    if (context.tokenCount === undefined) {
      conversation.context.tokenCount = this.estimateTokens(conversation);
    }

    logger.debug('Context updated', {
      conversationId,
      topics: conversation.context.topics,
      tokenCount: conversation.context.tokenCount,
    });
  }

  /**
   * Clear conversation
   */
  clearConversation(conversationId: string): void {
    this.conversations.delete(conversationId);
    logger.info('Conversation cleared', { conversationId });
  }

  /**
   * Prune old messages from conversation
   */
  pruneMessages(conversationId: string, keepCount: number): void {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) {
      throw new Error(`Conversation ${conversationId} not found`);
    }

    if (conversation.messages.length <= keepCount) {
      return; // No pruning needed
    }

    // Keep last N messages
    const prunedMessages = conversation.messages.slice(0, -keepCount);
    const keptMessages = conversation.messages.slice(-keepCount);

    // Create summary of pruned messages using Cohere (if available)
    const summary = this.summarizeMessages(prunedMessages);

    // Update conversation
    conversation.messages = keptMessages;
    conversation.context.summary = summary;
    conversation.context.lastUpdated = new Date();
    conversation.context.tokenCount = this.estimateTokens(conversation);

    logger.info('Messages pruned', {
      conversationId,
      prunedCount: prunedMessages.length,
      keptCount: keptMessages.length,
      newTokenCount: conversation.context.tokenCount,
    });
  }

  /**
   * Check if conversation needs pruning based on token count
   */
  needsPruning(conversationId: string): boolean {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) {
      return false;
    }

    const tokenCount = this.estimateTokens(conversation);
    return tokenCount > config.context.maxTokens;
  }

  /**
   * Auto-prune conversation if token limit exceeded
   */
  autoPruneIfNeeded(conversationId: string): void {
    if (this.needsPruning(conversationId)) {
      logger.info('Auto-pruning conversation', { conversationId });
      this.pruneMessages(conversationId, 20); // Keep last 20 messages
    }
  }

  /**
   * Create new conversation
   */
  createConversation(): string {
    const id = uuidv4();
    const conversation = createConversation(id);
    this.conversations.set(id, conversation);

    logger.info('Conversation created', { conversationId: id });
    return id;
  }

  /**
   * Get conversation by ID
   */
  getConversation(conversationId: string): Conversation | undefined {
    return this.conversations.get(conversationId);
  }

  /**
   * End conversation
   */
  endConversation(conversationId: string): void {
    const conversation = this.conversations.get(conversationId);
    if (conversation) {
      conversation.status = 'ended';
      conversation.updatedAt = new Date();
      logger.info('Conversation ended', { conversationId });
    }
  }

  /**
   * Estimate token count for conversation
   */
  estimateTokens(conversation: Conversation): number {
    // Rough estimate: 4 characters per token
    const totalChars = conversation.messages.reduce(
      (sum, msg) => sum + msg.content.length,
      0
    );
    return Math.ceil(totalChars / 4);
  }

  /**
   * Extract topics from messages
   */
  private extractTopics(messages: any[]): string[] {
    const topics = new Set<string>();
    const topicKeywords = new Map<string, number>(); // word -> frequency

    messages.forEach(msg => {
      // Simple keyword extraction (in production, use NLP)
      const words = msg.content.toLowerCase().split(/\s+/);
      words.forEach(word => {
        // Keep words longer than 5 characters as potential topics
        if (word.length > 5 && !this.isCommonWord(word)) {
          const count = topicKeywords.get(word) || 0;
          topicKeywords.set(word, count + 1);
        }
      });
    });

    // Sort by frequency and take top topics
    const sortedTopics = Array.from(topicKeywords.entries())
      .sort((a, b) => b[1] - a[1])
      .map(([word]) => word)
      .slice(0, 10);

    return sortedTopics;
  }

  /**
   * Detect context shift in conversation
   */
  detectContextShift(conversationId: string): boolean {
    const conversation = this.conversations.get(conversationId);
    if (!conversation || conversation.messages.length < 4) {
      return false;
    }

    // Compare topics from recent messages vs older messages
    const recentMessages = conversation.messages.slice(-3);
    const olderMessages = conversation.messages.slice(-6, -3);

    const recentTopics = new Set(this.extractTopics(recentMessages));
    const olderTopics = new Set(this.extractTopics(olderMessages));

    // Calculate topic overlap
    const overlap = Array.from(recentTopics).filter(topic => olderTopics.has(topic));
    const overlapRatio = overlap.length / Math.max(recentTopics.size, 1);

    // Context shift if less than 30% overlap
    const hasShift = overlapRatio < 0.3;

    if (hasShift) {
      logger.debug('Context shift detected', {
        conversationId,
        overlapRatio,
        recentTopics: Array.from(recentTopics),
        olderTopics: Array.from(olderTopics),
      });
    }

    return hasShift;
  }

  /**
   * Check if word is common (should be filtered out)
   */
  private isCommonWord(word: string): boolean {
    const commonWords = new Set([
      'the', 'and', 'for', 'that', 'this', 'with', 'from', 'have',
      'would', 'could', 'should', 'about', 'which', 'their', 'there',
    ]);
    return commonWords.has(word);
  }

  /**
   * Summarize messages
   */
  private summarizeMessages(messages: any[]): string {
    if (messages.length === 0) return '';

    // Simple summary: first sentence of each message
    const summaries = messages
      .map(msg => {
        const firstSentence = msg.content.split(/[.!?]/)[0];
        return firstSentence.substring(0, 100);
      })
      .slice(0, 5); // Max 5 summaries

    return `Previous conversation: ${summaries.join('. ')}.`;
  }

  /**
   * Start cleanup job for expired sessions
   */
  private startCleanupJob(): void {
    // Run cleanup every 5 minutes
    this.cleanupInterval = setInterval(() => {
      this.cleanupExpiredSessions();
    }, 5 * 60 * 1000);

    logger.info('Cleanup job started');
  }

  /**
   * Cleanup expired sessions
   */
  private cleanupExpiredSessions(): void {
    const now = Date.now();
    const timeoutMs = config.session.timeoutMs;
    let cleanedCount = 0;

    for (const [id, conversation] of this.conversations.entries()) {
      const lastActivity = conversation.updatedAt.getTime();
      const elapsed = now - lastActivity;

      if (elapsed > timeoutMs && conversation.status === 'active') {
        this.conversations.delete(id);
        cleanedCount++;
      }
    }

    if (cleanedCount > 0) {
      logger.info('Expired sessions cleaned up', {
        count: cleanedCount,
        remaining: this.conversations.size,
      });
    }
  }

  /**
   * Stop cleanup job
   */
  destroy(): void {
    if (this.cleanupInterval) {
      clearInterval(this.cleanupInterval);
      this.cleanupInterval = null;
      logger.info('Cleanup job stopped');
    }
  }

  /**
   * Get statistics
   */
  getStats(): {
    totalConversations: number;
    activeConversations: number;
    endedConversations: number;
  } {
    let active = 0;
    let ended = 0;

    for (const conversation of this.conversations.values()) {
      if (conversation.status === 'active') {
        active++;
      } else {
        ended++;
      }
    }

    return {
      totalConversations: this.conversations.size,
      activeConversations: active,
      endedConversations: ended,
    };
  }
}
