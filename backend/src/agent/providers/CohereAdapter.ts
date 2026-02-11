import { CohereClient } from 'cohere-ai';
import { IProvider } from './ProviderInterface';
import { ChatMessage, CompletionOptions, CompletionResult } from '../core/types';
import { ProviderError, RateLimitError, TimeoutError } from '../../utils/errors';
import { logger } from '../../utils/logger';
import { config } from '../../utils/config';

/**
 * Cohere API adapter implementing IProvider interface
 *
 * Provides integration with Cohere's chat and generate APIs
 * with error handling, rate limiting, and timeout support.
 */
export class CohereAdapter implements IProvider {
  private client: CohereClient;
  private readonly model: string;
  private readonly maxRetries: number = 3;
  private readonly initialRetryDelay: number = 1000; // 1 second

  constructor(apiKey?: string, model?: string) {
    const key = apiKey || config.cohere.apiKey;
    this.model = model || config.cohere.model;

    if (!key) {
      throw new ProviderError(
        'Cohere API key is required',
        'MISSING_API_KEY'
      );
    }

    this.client = new CohereClient({
      token: key,
    });

    logger.info('CohereAdapter initialized', { model: this.model });
  }

  /**
   * Generate completion from prompt
   */
  async complete(prompt: string, options: CompletionOptions = {}): Promise<CompletionResult> {
    const startTime = Date.now();

    try {
      const response = await this.withRetry(async () => {
        return await this.withTimeout(
          this.client.generate({
            prompt,
            model: this.model,
            temperature: options.temperature ?? config.cohere.temperature,
            maxTokens: options.maxTokens ?? config.cohere.maxTokens,
            stopSequences: options.stopSequences,
            p: options.topP,
            frequencyPenalty: options.frequencyPenalty,
            presencePenalty: options.presencePenalty,
          }),
          10000 // 10 second timeout
        );
      });

      const latency = Date.now() - startTime;
      logger.debug('Cohere generate completed', { latency, model: this.model });

      return {
        text: response.generations[0].text,
        finishReason: this.mapFinishReason(response.generations[0].finishReason),
        usage: {
          promptTokens: 0, // Cohere doesn't provide this in generate
          completionTokens: 0,
          totalTokens: 0,
        },
        raw: response,
      };
    } catch (error) {
      logger.error('Cohere generate failed', { error, prompt: prompt.substring(0, 100) });
      throw this.handleError(error);
    }
  }

  /**
   * Chat-style completion with message history
   */
  async chat(messages: ChatMessage[], options: CompletionOptions = {}): Promise<CompletionResult> {
    const startTime = Date.now();

    try {
      // Extract system message (preamble) and conversation history
      const systemMessage = messages.find(m => m.role === 'system');
      const conversationMessages = messages.filter(m => m.role !== 'system');

      // Get the last user message
      const lastUserMessage = conversationMessages[conversationMessages.length - 1];
      if (!lastUserMessage || lastUserMessage.role !== 'user') {
        throw new ProviderError(
          'Last message must be from user',
          'INVALID_MESSAGE_SEQUENCE'
        );
      }

      // Build chat history (all messages except the last user message)
      const chatHistory = conversationMessages.slice(0, -1).map(msg => ({
        role: msg.role === 'user' ? 'USER' : 'CHATBOT',
        message: msg.content,
      }));

      const response = await this.withRetry(async () => {
        return await this.withTimeout(
          this.client.chat({
            message: lastUserMessage.content,
            model: this.model,
            preamble: systemMessage?.content,
            chatHistory: chatHistory.length > 0 ? chatHistory : undefined,
            temperature: options.temperature ?? config.cohere.temperature,
            maxTokens: options.maxTokens ?? config.cohere.maxTokens,
            stopSequences: options.stopSequences,
            p: options.topP,
            frequencyPenalty: options.frequencyPenalty,
            presencePenalty: options.presencePenalty,
          }),
          10000 // 10 second timeout
        );
      });

      const latency = Date.now() - startTime;
      logger.debug('Cohere chat completed', {
        latency,
        model: this.model,
        messageCount: messages.length
      });

      return {
        text: response.text,
        finishReason: this.mapFinishReason(response.finishReason),
        usage: {
          promptTokens: response.meta?.tokens?.inputTokens || 0,
          completionTokens: response.meta?.tokens?.outputTokens || 0,
          totalTokens: (response.meta?.tokens?.inputTokens || 0) + (response.meta?.tokens?.outputTokens || 0),
        },
        raw: response,
      };
    } catch (error) {
      logger.error('Cohere chat failed', {
        error,
        messageCount: messages.length
      });
      throw this.handleError(error);
    }
  }

  /**
   * Check provider health
   */
  async healthCheck(): Promise<boolean> {
    try {
      // Use chat API instead of deprecated generate API
      await this.withTimeout(
        this.client.chat({
          message: 'test',
          model: this.model,
          maxTokens: 1,
        }),
        5000 // 5 second timeout for health check
      );
      return true;
    } catch (error) {
      logger.warn('Cohere health check failed', { error });
      return false;
    }
  }

  /**
   * Get provider name
   */
  getName(): string {
    return 'cohere';
  }

  /**
   * Retry logic with exponential backoff
   */
  private async withRetry<T>(fn: () => Promise<T>): Promise<T> {
    let lastError: any;
    let delay = this.initialRetryDelay;

    for (let attempt = 0; attempt < this.maxRetries; attempt++) {
      try {
        return await fn();
      } catch (error: any) {
        lastError = error;

        // Don't retry on certain errors
        if (this.isNonRetryableError(error)) {
          throw error;
        }

        // Check if it's a rate limit error
        if (this.isRateLimitError(error)) {
          logger.warn(`Rate limit hit, retrying after ${delay}ms`, { attempt });
          await this.sleep(delay);
          delay *= 2; // Exponential backoff
          continue;
        }

        // For other errors, only retry if not the last attempt
        if (attempt < this.maxRetries - 1) {
          logger.warn(`Request failed, retrying after ${delay}ms`, { attempt, error });
          await this.sleep(delay);
          delay *= 2;
          continue;
        }

        throw error;
      }
    }

    throw lastError;
  }

  /**
   * Add timeout to promise
   */
  private async withTimeout<T>(promise: Promise<T>, timeoutMs: number): Promise<T> {
    const timeoutPromise = new Promise<never>((_, reject) => {
      setTimeout(() => {
        reject(new TimeoutError(
          `Request timed out after ${timeoutMs}ms`,
          timeoutMs
        ));
      }, timeoutMs);
    });

    return Promise.race([promise, timeoutPromise]);
  }

  /**
   * Check if error is a rate limit error
   */
  private isRateLimitError(error: any): boolean {
    return (
      error?.statusCode === 429 ||
      error?.message?.toLowerCase().includes('rate limit') ||
      error?.message?.toLowerCase().includes('too many requests')
    );
  }

  /**
   * Check if error should not be retried
   */
  private isNonRetryableError(error: any): boolean {
    const nonRetryableStatusCodes = [400, 401, 403, 404];
    return nonRetryableStatusCodes.includes(error?.statusCode);
  }

  /**
   * Handle and transform errors
   */
  private handleError(error: any): Error {
    if (error instanceof TimeoutError) {
      return error;
    }

    if (this.isRateLimitError(error)) {
      return new RateLimitError(
        'Cohere API rate limit exceeded. Please wait and try again.',
        error.retryAfter
      );
    }

    const statusCode = error?.statusCode || error?.status;
    const message = error?.message || 'Unknown Cohere API error';

    if (statusCode === 401) {
      return new ProviderError(
        'Invalid Cohere API key',
        'INVALID_API_KEY',
        statusCode
      );
    }

    if (statusCode === 403) {
      return new ProviderError(
        'Access forbidden. Check your Cohere API permissions.',
        'FORBIDDEN',
        statusCode
      );
    }

    if (statusCode >= 500) {
      return new ProviderError(
        'Cohere API service error. Please try again later.',
        'SERVICE_ERROR',
        statusCode
      );
    }

    return new ProviderError(
      message,
      'PROVIDER_ERROR',
      statusCode,
      error
    );
  }

  /**
   * Map Cohere finish reason to standard format
   */
  private mapFinishReason(reason?: string): 'complete' | 'length' | 'stop' | 'tool_calls' {
    if (!reason) return 'complete';

    switch (reason.toLowerCase()) {
      case 'complete':
      case 'stop':
        return 'complete';
      case 'max_tokens':
      case 'length':
        return 'length';
      default:
        return 'complete';
    }
  }

  /**
   * Sleep utility
   */
  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}
