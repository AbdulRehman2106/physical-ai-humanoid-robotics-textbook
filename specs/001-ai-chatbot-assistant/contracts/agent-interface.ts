/**
 * Agent Interface Definitions
 *
 * TypeScript interfaces for agent abstraction following OpenAI Agent SDK patterns.
 * These interfaces enable provider-agnostic architecture and facilitate testing.
 *
 * @module agent-interface
 * @version 1.0.0
 */

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
   * @throws {AgentError} If processing fails
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
   * @throws {ProviderError} If API call fails
   */
  complete(prompt: string, options: CompletionOptions): Promise<CompletionResult>;

  /**
   * Chat-style completion with message history
   *
   * @param messages - Array of chat messages
   * @param options - Completion options
   * @returns Promise resolving to completion result
   * @throws {ProviderError} If API call fails
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

/**
 * Tool abstraction interface (for future extensibility)
 *
 * Tools extend agent capabilities by providing executable functions
 * that can be called during conversation.
 */
export interface ITool {
  /** Tool name (unique identifier) */
  name: string;

  /** Human-readable tool description */
  description: string;

  /** JSON schema for tool parameters */
  parameters: ToolParameters;

  /**
   * Execute tool with provided arguments
   *
   * @param args - Tool arguments matching parameter schema
   * @returns Promise resolving to tool result
   * @throws {ToolError} If execution fails
   */
  execute(args: Record<string, any>): Promise<ToolResult>;

  /**
   * Validate tool arguments against schema
   *
   * @param args - Arguments to validate
   * @returns True if valid, false otherwise
   */
  validate(args: Record<string, any>): boolean;
}

/**
 * Context interface for conversation state
 *
 * Maintains conversation history and metadata for context-aware responses.
 */
export interface IContext {
  /** Unique conversation identifier */
  conversationId: string;

  /** Message history (chronologically ordered) */
  messages: ChatMessage[];

  /** Conversation summary (for pruned messages) */
  summary: string;

  /** Identified topics in conversation */
  topics: string[];

  /** Approximate token count */
  tokenCount?: number;

  /** Last update timestamp */
  lastUpdated?: Date;
}

/**
 * Agent response structure
 *
 * Encapsulates the agent's response with metadata.
 */
export interface AgentResponse {
  /** Response content (markdown format) */
  content: string;

  /** Detected response format */
  format: ResponseFormat;

  /** Response metadata */
  metadata: {
    /** Model used for generation */
    model: string;

    /** Token usage */
    tokens: number;

    /** Response latency in milliseconds */
    latency: number;

    /** Tool calls made (if any) */
    toolCalls?: ToolCall[];

    /** Whether response was truncated */
    truncated?: boolean;

    /** Finish reason */
    finishReason?: 'complete' | 'length' | 'stop' | 'error';
  };
}

/**
 * Chat message structure
 *
 * Represents a single message in conversation history.
 */
export interface ChatMessage {
  /** Message role */
  role: 'system' | 'user' | 'assistant' | 'tool';

  /** Message content */
  content: string;

  /** Optional message name (for tool messages) */
  name?: string;

  /** Optional tool call ID (for tool responses) */
  toolCallId?: string;
}

/**
 * Completion options
 *
 * Configuration for LLM completion requests.
 */
export interface CompletionOptions {
  /** Sampling temperature (0.0 - 1.0) */
  temperature?: number;

  /** Maximum tokens to generate */
  maxTokens?: number;

  /** Stop sequences */
  stopSequences?: string[];

  /** Enable streaming responses */
  stream?: boolean;

  /** Top-p sampling */
  topP?: number;

  /** Frequency penalty */
  frequencyPenalty?: number;

  /** Presence penalty */
  presencePenalty?: number;

  /** Tools available for calling */
  tools?: ITool[];
}

/**
 * Completion result
 *
 * Result from LLM completion request.
 */
export interface CompletionResult {
  /** Generated text */
  text: string;

  /** Finish reason */
  finishReason: 'complete' | 'length' | 'stop' | 'tool_calls';

  /** Token usage statistics */
  usage: {
    /** Tokens in prompt */
    promptTokens: number;

    /** Tokens in completion */
    completionTokens: number;

    /** Total tokens used */
    totalTokens: number;
  };

  /** Tool calls requested by model (if any) */
  toolCalls?: ToolCall[];

  /** Raw provider response (for debugging) */
  raw?: any;
}

/**
 * Tool call structure
 *
 * Represents a tool invocation requested by the model.
 */
export interface ToolCall {
  /** Unique tool call identifier */
  id: string;

  /** Tool name to invoke */
  toolName: string;

  /** Tool arguments (JSON object) */
  arguments: Record<string, any>;

  /** Tool execution result (populated after execution) */
  result?: ToolResult;
}

/**
 * Tool result structure
 *
 * Result from tool execution.
 */
export interface ToolResult {
  /** Whether execution succeeded */
  success: boolean;

  /** Result data (if successful) */
  data?: any;

  /** Error message (if failed) */
  error?: string;

  /** Execution time in milliseconds */
  executionTime?: number;
}

/**
 * Agent configuration
 *
 * Configuration for agent behavior and provider settings.
 */
export interface AgentConfig {
  /** Provider identifier */
  provider: string;

  /** Model name */
  model: string;

  /** System prompt template */
  systemPrompt: string;

  /** Default temperature */
  temperature: number;

  /** Default max tokens */
  maxTokens: number;

  /** Available tools */
  tools?: ITool[];

  /** Enable safety checks */
  safetyEnabled?: boolean;

  /** Enable context pruning */
  contextPruningEnabled?: boolean;

  /** Maximum context tokens before pruning */
  maxContextTokens?: number;
}

/**
 * Tool parameters schema
 *
 * JSON schema for tool parameter validation.
 */
export interface ToolParameters {
  /** Schema type (always 'object' for tool parameters) */
  type: 'object';

  /** Parameter property definitions */
  properties: Record<string, ParameterSchema>;

  /** Required parameter names */
  required: string[];

  /** Additional properties allowed */
  additionalProperties?: boolean;
}

/**
 * Parameter schema
 *
 * JSON schema for individual parameter.
 */
export interface ParameterSchema {
  /** Parameter type */
  type: 'string' | 'number' | 'integer' | 'boolean' | 'array' | 'object';

  /** Parameter description */
  description: string;

  /** Enum values (for string/number types) */
  enum?: any[];

  /** Array item schema (for array type) */
  items?: ParameterSchema;

  /** Object properties (for object type) */
  properties?: Record<string, ParameterSchema>;

  /** Minimum value (for number/integer) */
  minimum?: number;

  /** Maximum value (for number/integer) */
  maximum?: number;

  /** Minimum length (for string/array) */
  minLength?: number;

  /** Maximum length (for string/array) */
  maxLength?: number;

  /** Pattern (for string) */
  pattern?: string;

  /** Default value */
  default?: any;
}

/**
 * Response format types
 *
 * Classification of response formatting.
 */
export type ResponseFormat = 'plain' | 'structured' | 'code' | 'table';

/**
 * Agent error
 *
 * Error thrown by agent operations.
 */
export class AgentError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'AgentError';
  }
}

/**
 * Provider error
 *
 * Error thrown by provider operations.
 */
export class ProviderError extends Error {
  constructor(
    message: string,
    public code: string,
    public statusCode?: number,
    public details?: any
  ) {
    super(message);
    this.name = 'ProviderError';
  }
}

/**
 * Tool error
 *
 * Error thrown by tool operations.
 */
export class ToolError extends Error {
  constructor(
    message: string,
    public toolName: string,
    public details?: any
  ) {
    super(message);
    this.name = 'ToolError';
  }
}

/**
 * Runner interface
 *
 * Orchestrates agent execution and manages conversation flow.
 */
export interface IRunner {
  /**
   * Execute agent with user input
   *
   * @param conversationId - Conversation identifier
   * @param userInput - User message
   * @returns Promise resolving to agent response
   */
  run(conversationId: string, userInput: string): Promise<AgentResponse>;

  /**
   * Create new conversation
   *
   * @returns New conversation ID
   */
  createConversation(): string;

  /**
   * End conversation
   *
   * @param conversationId - Conversation to end
   */
  endConversation(conversationId: string): void;

  /**
   * Get conversation context
   *
   * @param conversationId - Conversation identifier
   * @returns Conversation context
   */
  getContext(conversationId: string): IContext | undefined;
}

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
