import { ChatMessage } from '../core/types';
import { getSystemPrompt, isStepByStepRequest } from './SystemPrompts';

/**
 * Prompt orchestrator for assembling messages with system prompts
 *
 * Handles injection of system prompts, context management,
 * and message formatting for LLM providers.
 */
export class PromptOrchestrator {
  /**
   * Assemble messages for LLM with system prompt and conversation history
   *
   * @param userInput - Current user message
   * @param conversationHistory - Previous messages in conversation
   * @param options - Optional configuration
   * @returns Array of formatted messages
   */
  assembleMessages(
    userInput: string,
    conversationHistory: ChatMessage[] = [],
    options?: {
      includeStepByStep?: boolean;
      customSystemPrompt?: string;
    }
  ): ChatMessage[] {
    const messages: ChatMessage[] = [];

    // Determine if step-by-step instructions should be included
    const needsStepByStep = options?.includeStepByStep || isStepByStepRequest(userInput);

    // Add system prompt
    const systemPrompt = options?.customSystemPrompt || getSystemPrompt({
      includeStepByStep: needsStepByStep,
    });

    messages.push({
      role: 'system',
      content: systemPrompt,
    });

    // Add conversation history (excluding any existing system messages)
    const historyWithoutSystem = conversationHistory.filter(msg => msg.role !== 'system');
    messages.push(...historyWithoutSystem);

    // Add current user input
    messages.push({
      role: 'user',
      content: userInput,
    });

    return messages;
  }

  /**
   * Extract context summary from conversation history
   *
   * @param messages - Conversation messages
   * @returns Summary string
   */
  extractContextSummary(messages: ChatMessage[]): string {
    if (messages.length === 0) return '';

    // Simple summary: concatenate recent messages
    const recentMessages = messages.slice(-5); // Last 5 messages
    const summary = recentMessages
      .map(msg => `${msg.role}: ${msg.content.substring(0, 100)}`)
      .join('\n');

    return summary;
  }

  /**
   * Estimate token count for messages
   *
   * @param messages - Array of messages
   * @returns Approximate token count
   */
  estimateTokenCount(messages: ChatMessage[]): number {
    // Rough estimate: 4 characters per token
    const totalChars = messages.reduce((sum, msg) => sum + msg.content.length, 0);
    return Math.ceil(totalChars / 4);
  }

  /**
   * Check if messages exceed token limit
   *
   * @param messages - Array of messages
   * @param maxTokens - Maximum allowed tokens
   * @returns True if exceeds limit
   */
  exceedsTokenLimit(messages: ChatMessage[], maxTokens: number): boolean {
    return this.estimateTokenCount(messages) > maxTokens;
  }

  /**
   * Prune messages to fit within token limit
   *
   * @param messages - Array of messages
   * @param maxTokens - Maximum allowed tokens
   * @param keepCount - Minimum number of recent messages to keep
   * @returns Pruned messages with summary
   */
  pruneMessages(
    messages: ChatMessage[],
    maxTokens: number,
    keepCount: number = 20
  ): ChatMessage[] {
    if (!this.exceedsTokenLimit(messages, maxTokens)) {
      return messages;
    }

    // Keep system messages and recent messages
    const systemMessages = messages.filter(msg => msg.role === 'system');
    const nonSystemMessages = messages.filter(msg => msg.role !== 'system');
    const recentMessages = nonSystemMessages.slice(-keepCount);
    const prunedMessages = nonSystemMessages.slice(0, -keepCount);

    // Create summary of pruned messages
    const summary = this.createPrunedSummary(prunedMessages);

    // Assemble final message list
    const result: ChatMessage[] = [
      ...systemMessages,
      {
        role: 'system',
        content: `Previous conversation summary: ${summary}`,
      },
      ...recentMessages,
    ];

    return result;
  }

  /**
   * Create summary of pruned messages
   *
   * @param messages - Messages to summarize
   * @returns Summary string
   */
  private createPrunedSummary(messages: ChatMessage[]): string {
    if (messages.length === 0) return 'No previous context.';

    // Extract topics and key points
    const topics = new Set<string>();
    const keyPoints: string[] = [];

    messages.forEach(msg => {
      // Simple topic extraction (in production, use NLP or LLM)
      const words = msg.content.toLowerCase().split(/\s+/);
      words.forEach(word => {
        if (word.length > 5) topics.add(word);
      });

      // Keep first sentence of each message as key point
      const firstSentence = msg.content.split(/[.!?]/)[0];
      if (firstSentence && firstSentence.length > 10) {
        keyPoints.push(firstSentence.substring(0, 100));
      }
    });

    const topicList = Array.from(topics).slice(0, 5).join(', ');
    const pointsList = keyPoints.slice(0, 3).join('. ');

    return `Topics discussed: ${topicList}. Key points: ${pointsList}.`;
  }
}
