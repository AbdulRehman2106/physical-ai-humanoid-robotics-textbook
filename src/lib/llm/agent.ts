/**
 * Agent Abstraction Layer
 *
 * OpenAI Agent SDK-compatible patterns with provider abstraction
 * Following Constitutional Principle VIII: AI Assistant Integration
 */

import { LLMProvider, Message, ChatCompletionResponse, ProviderError } from './types';

export interface AgentConfig {
  name: string;
  instructions: string; // System prompt
  provider: LLMProvider;
  temperature?: number;
  maxTokens?: number;
}

export interface AgentResponse {
  content: string;
  success: boolean;
  error?: string;
  usage?: {
    promptTokens: number;
    completionTokens: number;
    totalTokens: number;
  };
}

export class Agent {
  private config: AgentConfig;
  private conversationHistory: Message[] = [];

  constructor(config: AgentConfig) {
    this.config = config;

    // Add system prompt to conversation history
    this.conversationHistory.push({
      role: 'system',
      content: config.instructions,
    });
  }

  async run(userMessage: string): Promise<AgentResponse> {
    // Check if provider is configured
    if (!this.config.provider.isConfigured()) {
      return {
        content: "I'm sorry, but the AI assistant is not configured. Please contact the administrator to set up the API key.",
        success: false,
        error: 'Provider not configured',
      };
    }

    // Add user message to history
    this.conversationHistory.push({
      role: 'user',
      content: userMessage,
    });

    try {
      // Get response from provider
      const response = await this.config.provider.chat({
        messages: this.conversationHistory,
        temperature: this.config.temperature,
        maxTokens: this.config.maxTokens,
      });

      // Add assistant response to history
      this.conversationHistory.push({
        role: 'assistant',
        content: response.content,
      });

      return {
        content: response.content,
        success: true,
        usage: response.usage,
      };
    } catch (error) {
      // Graceful error handling (Constitutional requirement)
      const errorMessage = error instanceof ProviderError
        ? error.message
        : 'An unexpected error occurred. Please try again.';

      return {
        content: "I apologize, but I'm having trouble processing your request right now. Please try again in a moment.",
        success: false,
        error: errorMessage,
      };
    }
  }

  // Clear conversation history (for new conversation)
  reset(): void {
    this.conversationHistory = [
      {
        role: 'system',
        content: this.config.instructions,
      },
    ];
  }

  // Get conversation history (for debugging/logging)
  getHistory(): Message[] {
    return [...this.conversationHistory];
  }
}

// Runner class for managing agent execution
export class Runner {
  private agent: Agent;

  constructor(agent: Agent) {
    this.agent = agent;
  }

  async execute(userMessage: string): Promise<AgentResponse> {
    const startTime = Date.now();

    const response = await this.agent.run(userMessage);

    const duration = Date.now() - startTime;

    // Log performance (Constitutional requirement: < 2s target)
    if (duration > 2000) {
      console.warn(`Agent response took ${duration}ms (target: < 2000ms)`);
    }

    return response;
  }

  reset(): void {
    this.agent.reset();
  }
}

// Factory function for creating configured agent
export function createTextbookAgent(provider: LLMProvider): Agent {
  return new Agent({
    name: 'Physical AI Textbook Assistant',
    instructions: `You are a helpful AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to help students learn about:
- Physical AI fundamentals and embodied intelligence
- ROS 2 development (nodes, topics, services, actions, lifecycle)
- Robot simulation (Gazebo, NVIDIA Isaac Sim)
- Vision-Language-Action (VLA) models
- Sim-to-real transfer techniques
- Error handling and robustness in robotics
- Building autonomous humanoid systems

Guidelines:
- Provide clear, accurate answers based on the textbook content
- If you're unsure about something, admit it explicitly (don't hallucinate)
- Use examples and analogies to explain complex concepts
- Encourage hands-on learning and experimentation
- Reference specific chapters when relevant
- Be encouraging and supportive of the learning process

Keep responses concise but informative. Focus on helping students understand concepts and solve problems.`,
    provider,
    temperature: 0.7,
    maxTokens: 1000,
  });
}
