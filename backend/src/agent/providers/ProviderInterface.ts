import { ChatMessage, CompletionOptions, CompletionResult } from '../core/types';

/**
 * Provider abstraction interface
 *
 * Abstracts LLM provider (Cohere, OpenAI, Anthropic, etc.) to enable
 * provider-agnostic agent implementation.
 */
export interface IProvider {
  /**
   * Generate completion from prompt
   *
   * @param prompt - Input prompt text
   * @param options - Completion options
   * @returns Promise resolving to completion result
   */
  complete(prompt: string, options: CompletionOptions): Promise<CompletionResult>;

  /**
   * Chat-style completion with message history
   *
   * @param messages - Array of chat messages
   * @param options - Completion options
   * @returns Promise resolving to completion result
   */
  chat(messages: ChatMessage[], options: CompletionOptions): Promise<CompletionResult>;

  /**
   * Check provider health
   *
   * @returns Promise resolving to true if provider is available
   */
  healthCheck(): Promise<boolean>;

  /**
   * Get provider name
   *
   * @returns Provider identifier (e.g., 'cohere', 'openai')
   */
  getName(): string;
}
