import { IAgent } from './Agent';
import { IProvider } from '../providers/ProviderInterface';
import { IContext, AgentResponse, AgentConfig } from './types';
import { detectFormat, detectFormatMetadata } from '../../models/ResponseFormat';
import { logger } from '../../utils/logger';
import { config } from '../../utils/config';

/**
 * Chat agent implementation
 *
 * Processes user input and generates responses using the configured LLM provider.
 * Implements the IAgent interface following OpenAI Agent SDK patterns.
 */
export class ChatAgent implements IAgent {
  private provider: IProvider;
  private agentConfig: AgentConfig;

  constructor(provider: IProvider, agentConfig?: Partial<AgentConfig>) {
    this.provider = provider;
    this.agentConfig = {
      provider: provider.getName(),
      model: config.cohere.model,
      systemPrompt: '', // Will be set by PromptOrchestrator
      temperature: config.cohere.temperature,
      maxTokens: config.cohere.maxTokens,
      safetyEnabled: true,
      contextPruningEnabled: true,
      maxContextTokens: config.context.maxTokens,
      ...agentConfig,
    };

    logger.info('ChatAgent initialized', {
      provider: this.agentConfig.provider,
      model: this.agentConfig.model,
    });
  }

  /**
   * Process user input and generate response
   */
  async run(input: string, context: IContext): Promise<AgentResponse> {
    const startTime = Date.now();

    try {
      logger.debug('Agent processing input', {
        conversationId: context.conversationId,
        inputLength: input.length,
        messageCount: context.messages.length,
      });

      // Call provider with messages
      const result = await this.provider.chat(context.messages, {
        temperature: this.agentConfig.temperature,
        maxTokens: this.agentConfig.maxTokens,
      });

      const latency = Date.now() - startTime;

      // Detect response format
      const format = detectFormat(result.text);
      const formatMetadata = detectFormatMetadata(result.text);

      // Build agent response
      const response: AgentResponse = {
        content: result.text,
        format,
        metadata: {
          model: this.agentConfig.model,
          tokens: result.usage.totalTokens,
          latency,
          finishReason: result.finishReason,
          toolCalls: result.toolCalls,
        },
      };

      logger.info('Agent response generated', {
        conversationId: context.conversationId,
        format,
        tokens: result.usage.totalTokens,
        latency,
        hasCodeBlocks: formatMetadata.hasCodeBlocks,
      });

      return response;
    } catch (error) {
      logger.error('Agent run failed', {
        error,
        conversationId: context.conversationId,
      });
      throw error;
    }
  }

  /**
   * Get agent configuration
   */
  getConfig(): AgentConfig {
    return { ...this.agentConfig };
  }

  /**
   * Update agent configuration
   */
  updateConfig(config: Partial<AgentConfig>): void {
    this.agentConfig = {
      ...this.agentConfig,
      ...config,
    };

    logger.info('Agent configuration updated', {
      updates: Object.keys(config),
    });
  }
}
