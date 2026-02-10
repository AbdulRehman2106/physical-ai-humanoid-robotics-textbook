import { ToolParameters, ToolResult } from './types';

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
