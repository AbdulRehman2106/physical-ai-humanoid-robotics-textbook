/**
 * LLM Provider Abstraction Layer
 *
 * Provides a unified interface for different LLM providers (Cohere, OpenAI, etc.)
 * Following Constitutional Principle VIII: AI Assistant Integration
 */

export interface Message {
  role: 'system' | 'user' | 'assistant';
  content: string;
}

export interface ChatCompletionRequest {
  messages: Message[];
  temperature?: number;
  maxTokens?: number;
  stream?: boolean;
}

export interface ChatCompletionResponse {
  content: string;
  finishReason: 'complete' | 'max_tokens' | 'error';
  usage?: {
    promptTokens: number;
    completionTokens: number;
    totalTokens: number;
  };
}

export interface LLMProvider {
  name: string;
  chat(request: ChatCompletionRequest): Promise<ChatCompletionResponse>;
  isConfigured(): boolean;
}

export class ProviderError extends Error {
  constructor(
    message: string,
    public provider: string,
    public originalError?: Error
  ) {
    super(message);
    this.name = 'ProviderError';
  }
}
