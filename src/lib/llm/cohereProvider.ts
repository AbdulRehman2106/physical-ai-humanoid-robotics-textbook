/**
 * Cohere LLM Provider Implementation
 *
 * Implements the LLM provider interface for Cohere API
 * Following Constitutional Principle VIII: AI Assistant Integration
 */

import {
  LLMProvider,
  ChatCompletionRequest,
  ChatCompletionResponse,
  Message,
  ProviderError,
} from './types';

export class CohereProvider implements LLMProvider {
  name = 'cohere';
  private apiKey: string | undefined;
  private baseUrl = 'https://api.cohere.ai/v1';

  constructor(apiKey?: string) {
    // Try environment variable first, then constructor parameter
    this.apiKey = apiKey || process.env.COHERE_API_KEY || (typeof window !== 'undefined' ? (window as any).COHERE_API_KEY : undefined);
  }

  isConfigured(): boolean {
    return !!this.apiKey;
  }

  async chat(request: ChatCompletionRequest): Promise<ChatCompletionResponse> {
    if (!this.isConfigured()) {
      throw new ProviderError(
        'Cohere API key not configured. Please set COHERE_API_KEY environment variable.',
        this.name
      );
    }

    try {
      // Convert messages to Cohere format
      const { systemPrompt, chatHistory, userMessage } = this.formatMessages(request.messages);

      const response = await fetch(`${this.baseUrl}/chat`, {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${this.apiKey}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          model: 'command-r-plus', // Using Command-R+ for best quality
          message: userMessage,
          chat_history: chatHistory,
          preamble: systemPrompt,
          temperature: request.temperature ?? 0.7,
          max_tokens: request.maxTokens ?? 1000,
          stream: false, // Non-streaming for simplicity
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(
          `Cohere API error: ${response.status} ${response.statusText} - ${JSON.stringify(errorData)}`
        );
      }

      const data = await response.json();

      return {
        content: data.text || '',
        finishReason: data.finish_reason === 'COMPLETE' ? 'complete' : 'max_tokens',
        usage: data.meta?.tokens ? {
          promptTokens: data.meta.tokens.input_tokens || 0,
          completionTokens: data.meta.tokens.output_tokens || 0,
          totalTokens: (data.meta.tokens.input_tokens || 0) + (data.meta.tokens.output_tokens || 0),
        } : undefined,
      };
    } catch (error) {
      throw new ProviderError(
        `Failed to get response from Cohere: ${error.message}`,
        this.name,
        error
      );
    }
  }

  private formatMessages(messages: Message[]): {
    systemPrompt: string;
    chatHistory: Array<{ role: string; message: string }>;
    userMessage: string;
  } {
    // Extract system prompt (first system message)
    const systemMessage = messages.find(m => m.role === 'system');
    const systemPrompt = systemMessage?.content || '';

    // Get conversation history (excluding system and last user message)
    const conversationMessages = messages.filter(m => m.role !== 'system');
    const lastUserMessage = conversationMessages[conversationMessages.length - 1];
    const historyMessages = conversationMessages.slice(0, -1);

    // Convert to Cohere chat history format
    const chatHistory = historyMessages.map(msg => ({
      role: msg.role === 'user' ? 'USER' : 'CHATBOT',
      message: msg.content,
    }));

    return {
      systemPrompt,
      chatHistory,
      userMessage: lastUserMessage?.content || '',
    };
  }
}

// Factory function for easy instantiation
export function createCohereProvider(apiKey?: string): LLMProvider {
  return new CohereProvider(apiKey);
}
