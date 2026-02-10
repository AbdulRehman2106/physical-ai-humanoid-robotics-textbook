# Data Model: AI Chatbot Assistant

**Feature**: 001-ai-chatbot-assistant
**Date**: 2026-02-10
**Purpose**: Define data entities and their relationships for the chatbot system

---

## Overview

The data model consists of four core entities that support conversational AI interactions:
1. **Conversation** - Session container for user-assistant exchanges
2. **Message** - Individual communication units within a conversation
3. **Context** - Accumulated understanding and metadata
4. **ResponseFormat** - Structure metadata for formatted responses

---

## Entity Definitions

### 1. Conversation

Represents a complete session of interaction between user and chatbot.

**TypeScript Definition**:
```typescript
interface Conversation {
  id: string;                    // Unique session identifier (UUID v4)
  createdAt: Date;               // Session start time
  updatedAt: Date;               // Last activity time
  messages: Message[];           // Ordered message history
  context: Context;              // Accumulated context
  status: 'active' | 'ended';    // Session status
}
```

**Field Descriptions**:
- `id`: UUID v4 format, generated on conversation creation
- `createdAt`: Timestamp when conversation was initiated
- `updatedAt`: Timestamp of last message or activity
- `messages`: Array of Message objects, chronologically ordered
- `context`: Context object containing conversation metadata
- `status`: Lifecycle state of the conversation

**Validation Rules**:
- `id` MUST be valid UUID v4 format
- `messages` array MUST be ordered chronologically (oldest to newest)
- `context` MUST be updated after each message exchange
- `updatedAt` MUST be >= `createdAt`
- Sessions with status 'active' expire after 30 minutes of inactivity

**State Transitions**:
```
[Created] → active
active → ended (user ends session OR timeout)
ended → [No further transitions]
```

**Relationships**:
- One Conversation has many Messages (1:N)
- One Conversation has one Context (1:1)

---

### 2. Message

Individual user query or assistant response within a conversation.

**TypeScript Definition**:
```typescript
interface Message {
  id: string;                    // Unique message identifier (UUID v4)
  conversationId: string;        // Parent conversation reference
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text
  timestamp: Date;               // Message creation time
  metadata?: {
    model?: string;              // Cohere model used (for assistant messages)
    tokens?: number;             // Token count
    latency?: number;            // Response time in milliseconds
  };
}
```

**Field Descriptions**:
- `id`: UUID v4 format, unique per message
- `conversationId`: Foreign key reference to parent Conversation
- `role`: Identifies sender ('user' for human input, 'assistant' for AI response)
- `content`: The actual message text (supports markdown formatting)
- `timestamp`: When the message was created
- `metadata`: Optional performance and tracking data (populated for assistant messages)

**Validation Rules**:
- `content` MUST NOT be empty string
- `role` MUST be exactly 'user' or 'assistant'
- `conversationId` MUST reference an existing Conversation
- `timestamp` MUST be within conversation's createdAt and updatedAt range
- `metadata.model` SHOULD be populated for assistant messages
- `metadata.tokens` SHOULD be positive integer if present
- `metadata.latency` SHOULD be positive integer (milliseconds) if present

**Relationships**:
- Many Messages belong to one Conversation (N:1)

**Example**:
```typescript
const userMessage: Message = {
  id: '550e8400-e29b-41d4-a716-446655440001',
  conversationId: '550e8400-e29b-41d4-a716-446655440000',
  role: 'user',
  content: 'How do I implement error handling in Python?',
  timestamp: new Date('2026-02-10T12:00:00Z'),
};

const assistantMessage: Message = {
  id: '550e8400-e29b-41d4-a716-446655440002',
  conversationId: '550e8400-e29b-41d4-a716-446655440000',
  role: 'assistant',
  content: 'Here\'s how to implement error handling in Python:\n\n1. Use try-except blocks...',
  timestamp: new Date('2026-02-10T12:00:05Z'),
  metadata: {
    model: 'command-r',
    tokens: 150,
    latency: 1200,
  },
};
```

---

### 3. Context

Accumulated understanding and metadata for a conversation.

**TypeScript Definition**:
```typescript
interface Context {
  conversationId: string;        // Parent conversation reference
  summary: string;               // Accumulated context summary
  topics: string[];              // Identified topics in conversation
  lastUpdated: Date;             // Last context update timestamp
  tokenCount: number;            // Approximate token usage
}
```

**Field Descriptions**:
- `conversationId`: Foreign key reference to parent Conversation
- `summary`: Natural language summary of conversation history (used when pruning old messages)
- `topics`: Array of identified topics/themes in the conversation
- `lastUpdated`: Timestamp of last context update
- `tokenCount`: Estimated total tokens in conversation (for context window management)

**Validation Rules**:
- `conversationId` MUST reference an existing Conversation
- `tokenCount` MUST be non-negative integer
- `topics` array SHOULD contain 1-10 topics (empty array valid for new conversations)
- `summary` updated when messages are pruned from context window
- `lastUpdated` MUST be <= current time

**Update Triggers**:
- After each message exchange
- When context pruning occurs (token limit exceeded)
- When topics are extracted from conversation

**Token Management**:
- `tokenCount` estimated as: total characters / 4 (rough approximation)
- When `tokenCount` exceeds 100,000 tokens, trigger context pruning
- Pruning: Keep last 20 messages, summarize older messages into `summary` field

**Example**:
```typescript
const context: Context = {
  conversationId: '550e8400-e29b-41d4-a716-446655440000',
  summary: 'User asked about Python error handling. Discussed try-except blocks and best practices.',
  topics: ['Python', 'error handling', 'exceptions', 'try-except'],
  lastUpdated: new Date('2026-02-10T12:00:05Z'),
  tokenCount: 1250,
};
```

---

### 4. ResponseFormat

Metadata describing the structure of assistant responses.

**TypeScript Definition**:
```typescript
type ResponseFormat =
  | 'plain'                      // Plain text, no special formatting
  | 'structured'                 // Headers, lists, sections
  | 'code'                       // Code blocks with syntax highlighting
  | 'table';                     // Tabular data

interface FormattedResponse {
  content: string;               // Raw response content
  format: ResponseFormat;        // Detected format type
  metadata: {
    hasCodeBlocks: boolean;      // Contains code blocks
    hasLists: boolean;           // Contains bullet/numbered lists
    hasHeaders: boolean;         // Contains markdown headers
  };
}
```

**Field Descriptions**:
- `content`: The actual response text (markdown format)
- `format`: Primary format classification
- `metadata`: Boolean flags for format detection

**Format Detection Logic**:
```typescript
function detectFormat(content: string): ResponseFormat {
  const hasCodeBlocks = /```[\s\S]*?```/.test(content);
  const hasHeaders = /^#{1,6}\s/m.test(content);
  const hasLists = /^[\*\-\+]\s|^\d+\.\s/m.test(content);
  const hasTables = /\|.*\|/.test(content);

  if (hasTables) return 'table';
  if (hasCodeBlocks) return 'code';
  if (hasHeaders || hasLists) return 'structured';
  return 'plain';
}
```

**Usage**:
- Frontend uses `format` to apply appropriate styling
- `metadata` flags enable conditional rendering (syntax highlighting, list formatting)
- Helps meet FR-003 (format responses with appropriate structure)

---

## Relationships Diagram

```
┌─────────────────────────────────────┐
│         Conversation                │
│  - id: string                       │
│  - createdAt: Date                  │
│  - updatedAt: Date                  │
│  - status: 'active' | 'ended'       │
└───────────┬─────────────────────────┘
            │
            │ 1:N
            │
┌───────────▼─────────────────────────┐
│           Message                   │
│  - id: string                       │
│  - conversationId: string (FK)      │
│  - role: 'user' | 'assistant'       │
│  - content: string                  │
│  - timestamp: Date                  │
│  - metadata?: {...}                 │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│         Conversation                │
└───────────┬─────────────────────────┘
            │
            │ 1:1
            │
┌───────────▼─────────────────────────┐
│           Context                   │
│  - conversationId: string (FK)      │
│  - summary: string                  │
│  - topics: string[]                 │
│  - lastUpdated: Date                │
│  - tokenCount: number               │
└─────────────────────────────────────┘
```

---

## Storage Strategy

### MVP (In-Memory)
- Conversations stored in `Map<string, Conversation>`
- No persistent storage
- Session cleanup every 5 minutes (remove expired conversations)
- Maximum 100 concurrent conversations

### Future (Persistent Storage)
- **Database**: PostgreSQL or MongoDB
- **Session Store**: Redis for active conversations
- **Archive**: S3 or similar for ended conversations
- **Schema Migration**: Alembic (Python) or TypeORM migrations (TypeScript)

---

## Data Flow

### 1. Create Conversation
```
POST /api/chat/conversations
    ↓
Generate UUID for conversation.id
    ↓
Initialize empty messages array
    ↓
Create initial Context (empty summary, no topics)
    ↓
Set status = 'active'
    ↓
Store in memory
    ↓
Return Conversation object
```

### 2. Send Message
```
POST /api/chat/conversations/{id}/messages
    ↓
Create Message object (role='user')
    ↓
Add to conversation.messages
    ↓
Update conversation.updatedAt
    ↓
Process with Agent
    ↓
Create assistant Message
    ↓
Add to conversation.messages
    ↓
Update Context (summary, topics, tokenCount)
    ↓
Check token limit → prune if needed
    ↓
Return assistant Message
```

### 3. Context Pruning
```
tokenCount > 100,000
    ↓
Keep last 20 messages
    ↓
Summarize older messages
    ↓
Create system message with summary
    ↓
Update Context.summary
    ↓
Recalculate tokenCount
```

---

## Validation & Constraints

### Conversation Constraints
- Maximum 1000 messages per conversation
- Maximum 30 minutes inactivity before expiration
- Maximum 100 concurrent active conversations (MVP limit)

### Message Constraints
- Maximum content length: 10,000 characters per message
- Minimum content length: 1 character
- No HTML tags allowed (sanitized on input)

### Context Constraints
- Maximum 10 topics tracked
- Summary maximum length: 1,000 characters
- Token count recalculated after each message

### Performance Targets
- Message creation: <50ms
- Context update: <100ms
- Context pruning: <500ms
- Conversation retrieval: <10ms

---

## Testing Considerations

### Unit Tests
- Validate UUID generation
- Test state transitions
- Verify validation rules
- Test token estimation accuracy

### Integration Tests
- Create conversation → send messages → verify context updates
- Test context pruning when token limit exceeded
- Verify session expiration logic
- Test concurrent conversation handling

### Load Tests
- 100 concurrent conversations
- 1000 messages per conversation
- Context pruning performance under load

---

## Summary

The data model provides a clean, type-safe foundation for the chatbot system:
- **Conversation**: Session container with lifecycle management
- **Message**: Individual exchanges with metadata tracking
- **Context**: Intelligent context management with token awareness
- **ResponseFormat**: Format detection for proper rendering

All entities support the functional requirements (FR-001 through FR-015) and success criteria (SC-001 through SC-010) defined in the specification.
