/**
 * Core types and interfaces for the agent system
 */

/**
 * Chat message structure
 */
export interface ChatMessage {
  role: 'system' | 'user' | 'assistant' | 'tool';
  content: string;
  name?: string;
  toolCallId?: string;
}

/**
 * Context interface for conversation state
 */
export interface IContext {
  conversationId: string;
  messages: ChatMessage[];
  summary: string;
  topics: string[];
  tokenCount?: number;
  lastUpdated?: Date;
}

/**
 * Response format types
 */
export type ResponseFormat = 'plain' | 'structured' | 'code' | 'table';

/**
 * Agent response structure
 */
export interface AgentResponse {
  content: string;
  format: ResponseFormat;
  metadata: {
    model: string;
    tokens: number;
    latency: number;
    toolCalls?: ToolCall[];
    truncated?: boolean;
    finishReason?: 'complete' | 'length' | 'stop' | 'tool_calls' | 'error';
  };
}

/**
 * Completion options
 */
export interface CompletionOptions {
  temperature?: number;
  maxTokens?: number;
  stopSequences?: string[];
  stream?: boolean;
  topP?: number;
  frequencyPenalty?: number;
  presencePenalty?: number;
}

/**
 * Completion result
 */
export interface CompletionResult {
  text: string;
  finishReason: 'complete' | 'length' | 'stop' | 'tool_calls';
  usage: {
    promptTokens: number;
    completionTokens: number;
    totalTokens: number;
  };
  toolCalls?: ToolCall[];
  raw?: any;
}

/**
 * Tool call structure
 */
export interface ToolCall {
  id: string;
  toolName: string;
  arguments: Record<string, any>;
  result?: ToolResult;
}

/**
 * Tool result structure
 */
export interface ToolResult {
  success: boolean;
  data?: any;
  error?: string;
  executionTime?: number;
}

/**
 * Agent configuration
 */
export interface AgentConfig {
  provider: string;
  model: string;
  systemPrompt: string;
  temperature: number;
  maxTokens: number;
  tools?: any[];
  safetyEnabled?: boolean;
  contextPruningEnabled?: boolean;
  maxContextTokens?: number;
}

/**
 * Tool parameters schema
 */
export interface ToolParameters {
  type: 'object';
  properties: Record<string, ParameterSchema>;
  required: string[];
  additionalProperties?: boolean;
}

/**
 * Parameter schema
 */
export interface ParameterSchema {
  type: 'string' | 'number' | 'integer' | 'boolean' | 'array' | 'object';
  description: string;
  enum?: any[];
  items?: ParameterSchema;
  properties?: Record<string, ParameterSchema>;
  minimum?: number;
  maximum?: number;
  minLength?: number;
  maxLength?: number;
  pattern?: string;
  default?: any;
}
