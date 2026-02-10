import { IContext, AgentResponse, AgentConfig } from './types';

/**
 * Agent abstraction interface (OpenAI Agent SDK pattern)
 *
 * The Agent is responsible for processing user input and generating responses
 * using the configured LLM provider.
 */
export interface IAgent {
  /**
   * Process user input and generate response
   *
   * @param input - User message content
   * @param context - Conversation context
   * @returns Promise resolving to agent response
   */
  run(input: string, context: IContext): Promise<AgentResponse>;

  /**
   * Get agent configuration
   *
   * @returns Current agent configuration
   */
  getConfig(): AgentConfig;

  /**
   * Update agent configuration
   *
   * @param config - Partial configuration to update
   */
  updateConfig(config: Partial<AgentConfig>): void;
}
