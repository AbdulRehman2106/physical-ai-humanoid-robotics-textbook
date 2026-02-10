/**
 * Conversation entity
 *
 * Represents a complete session of interaction between user and chatbot
 */

export interface Conversation {
  id: string;                    // UUID v4
  createdAt: Date;               // Session start time
  updatedAt: Date;               // Last activity time
  messages: Message[];           // Ordered message history
  context: Context;              // Accumulated context
  status: 'active' | 'ended';    // Session status
}

/**
 * Message entity
 *
 * Individual user query or assistant response within a conversation
 */
export interface Message {
  id: string;                    // UUID v4
  conversationId: string;        // Parent conversation reference
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text
  timestamp: Date;               // Message creation time
  metadata?: {
    model?: string;              // Cohere model used (for assistant)
    tokens?: number;             // Token count
    latency?: number;            // Response time (ms)
  };
}

/**
 * Context entity
 *
 * Accumulated understanding and metadata for a conversation
 */
export interface Context {
  conversationId: string;        // Parent conversation reference
  summary: string;               // Accumulated context summary
  topics: string[];              // Identified topics
  lastUpdated: Date;             // Last context update
  tokenCount: number;            // Approximate token usage
}

/**
 * Validation functions
 */

export function validateConversation(conversation: Partial<Conversation>): string[] {
  const errors: string[] = [];

  if (!conversation.id || !isValidUUID(conversation.id)) {
    errors.push('Conversation ID must be a valid UUID v4');
  }

  if (conversation.messages && conversation.messages.length > 1000) {
    errors.push('Conversation cannot exceed 1000 messages');
  }

  if (conversation.status && !['active', 'ended'].includes(conversation.status)) {
    errors.push('Status must be either "active" or "ended"');
  }

  return errors;
}

export function validateMessage(message: Partial<Message>): string[] {
  const errors: string[] = [];

  if (!message.id || !isValidUUID(message.id)) {
    errors.push('Message ID must be a valid UUID v4');
  }

  if (!message.conversationId || !isValidUUID(message.conversationId)) {
    errors.push('Conversation ID must be a valid UUID v4');
  }

  if (!message.content || message.content.trim().length === 0) {
    errors.push('Message content cannot be empty');
  }

  if (message.content && message.content.length > 10000) {
    errors.push('Message content cannot exceed 10,000 characters');
  }

  if (!message.role || !['user', 'assistant'].includes(message.role)) {
    errors.push('Role must be either "user" or "assistant"');
  }

  return errors;
}

export function validateContext(context: Partial<Context>): string[] {
  const errors: string[] = [];

  if (!context.conversationId || !isValidUUID(context.conversationId)) {
    errors.push('Conversation ID must be a valid UUID v4');
  }

  if (context.tokenCount !== undefined && context.tokenCount < 0) {
    errors.push('Token count cannot be negative');
  }

  if (context.topics && context.topics.length > 10) {
    errors.push('Cannot track more than 10 topics');
  }

  return errors;
}

/**
 * Helper function to validate UUID v4
 */
function isValidUUID(uuid: string): boolean {
  const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
  return uuidRegex.test(uuid);
}

/**
 * Factory functions
 */

export function createConversation(id: string): Conversation {
  const now = new Date();
  return {
    id,
    createdAt: now,
    updatedAt: now,
    messages: [],
    context: createContext(id),
    status: 'active',
  };
}

export function createMessage(
  id: string,
  conversationId: string,
  role: 'user' | 'assistant',
  content: string,
  metadata?: Message['metadata']
): Message {
  return {
    id,
    conversationId,
    role,
    content,
    timestamp: new Date(),
    metadata,
  };
}

export function createContext(conversationId: string): Context {
  return {
    conversationId,
    summary: '',
    topics: [],
    lastUpdated: new Date(),
    tokenCount: 0,
  };
}
