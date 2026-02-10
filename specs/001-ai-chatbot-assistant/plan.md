# Implementation Plan: AI Chatbot Assistant

**Branch**: `001-ai-chatbot-assistant` | **Date**: 2026-02-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-chatbot-assistant/spec.md`

## Summary

Build a general-purpose AI chatbot assistant that provides professional, context-aware conversational support for developers, students, and general users. The system will use Cohere as the LLM provider with an OpenAI Agent SDK-compatible architecture pattern, enabling provider-agnostic design. Core capabilities include technical Q&A, step-by-step explanations, multi-turn context maintenance, and safe boundary enforcement (refusing medical/legal advice and autonomous actions).

**Technical Approach**: Agent core with Cohere adapter layer, prompt orchestration system, and optional memory manager. Execution flow: User Input → System Prompt Injection → Agent Reasoning → Cohere Completion → Tool Routing → Final Response.

## Technical Context

**Language/Version**: TypeScript 5.x / Node.js 20.x (for agent runtime and API layer)
**Primary Dependencies**:
- `cohere-ai` SDK (Cohere API client)
- OpenAI Agent SDK patterns (agent abstraction, not the actual OpenAI SDK)
- Express.js or Fastify (API server)
- Zod (schema validation)

**Storage**: In-memory conversation state (session-based); no persistent database for MVP
**Testing**: Jest for unit tests; Supertest for API integration tests; manual conversation flow testing
**Target Platform**: Node.js server (Linux/Docker); Web UI (React/Next.js) as separate interface
**Project Type**: Web application (backend API + frontend UI)
**Performance Goals**:
- <2s response time for 95% of queries (per constitution)
- <5s for complex multi-turn responses
- Support 100 concurrent conversations

**Constraints**:
- Stateless by default (no long-term memory across sessions)
- API key via environment variable only (NEVER hardcoded)
- Graceful degradation on Cohere API failure
- No hallucinated facts; explicit uncertainty admission required

**Scale/Scope**:
- MVP: Single-user conversational interface
- 10-20 conversation turns per session typical
- English language only (initial version)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principle VIII: AI Assistant Integration ✅

- **Provider Abstraction**: ✅ Architecture uses Cohere adapter layer; LLM provider interchangeable
- **No Direct Provider Calls**: ✅ All completions routed through abstraction layer
- **Cohere as Primary**: ✅ Using Cohere API (Command/Command-R models)
- **OpenAI Agent SDK Patterns**: ✅ Agent, Runner, Tool abstraction patterns adopted
- **Graceful Fallback**: ✅ Error handling with user-friendly messages planned
- **No Hallucinations**: ✅ System prompts enforce uncertainty admission
- **Stateless Memory**: ✅ Conversational memory stateless by default
- **API Key Security**: ✅ Environment variable `COHERE_API_KEY` only

### Technical Accuracy (Principle I) ✅

- **Verified Claims**: ✅ Responses based on Cohere model training; no fabricated facts
- **Code Examples**: ✅ Agent can provide tested code examples
- **Authoritative Sources**: ✅ System prompts guide toward verified information

### Pedagogical Clarity (Principle II) ✅

- **Step-by-Step**: ✅ FR-007 requires step-by-step explanations
- **Clear Responses**: ✅ FR-012 requires clear, concise responses
- **Professional Tone**: ✅ FR-009 requires professional, neutral tone

### Professional Quality Standards (Principle V) ✅

- **Response Formatting**: ✅ FR-003 requires structured responses (headers, lists, code blocks)
- **Error Messages**: ✅ Educational and actionable error messages planned
- **Performance**: ✅ <2s response time target aligns with constitution

### Safety & Scope Boundaries ✅

- **Refuse Out-of-Scope**: ✅ FR-005, FR-006 enforce medical/legal/autonomous action refusal
- **Explicit Uncertainty**: ✅ FR-004 requires uncertainty admission
- **No Hidden Actions**: ✅ FR-015 prohibits background execution

**GATE RESULT**: ✅ **PASS** - All constitution principles satisfied. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-chatbot-assistant/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - Cohere API patterns, agent architecture
├── data-model.md        # Phase 1 output - Conversation, Message, Context entities
├── quickstart.md        # Phase 1 output - Setup and usage guide
├── contracts/           # Phase 1 output - API contracts
│   ├── chat-api.yaml    # OpenAPI spec for chat endpoints
│   └── agent-interface.ts  # TypeScript interfaces for agent abstraction
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── agent/
│   │   ├── core/
│   │   │   ├── Agent.ts           # Agent abstraction (OpenAI SDK pattern)
│   │   │   ├── Runner.ts          # Execution runner
│   │   │   └── Tool.ts            # Tool abstraction interface
│   │   ├── providers/
│   │   │   ├── CohereAdapter.ts   # Cohere API adapter
│   │   │   └── ProviderInterface.ts  # Provider abstraction
│   │   ├── prompts/
│   │   │   ├── SystemPrompts.ts   # System prompt templates
│   │   │   └── PromptOrchestrator.ts  # Prompt assembly logic
│   │   └── memory/
│   │       ├── ConversationMemory.ts  # Session-based memory
│   │       └── MemoryManager.ts   # Optional memory interface
│   ├── api/
│   │   ├── routes/
│   │   │   └── chat.ts            # Chat API endpoints
│   │   ├── middleware/
│   │   │   ├── errorHandler.ts    # Error handling middleware
│   │   │   └── validation.ts      # Request validation
│   │   └── server.ts              # Express/Fastify server setup
│   ├── models/
│   │   ├── Conversation.ts        # Conversation entity
│   │   ├── Message.ts             # Message entity
│   │   └── Context.ts             # Context entity
│   ├── services/
│   │   ├── ChatService.ts         # Business logic for chat
│   │   └── SafetyService.ts       # Safety boundary enforcement
│   └── utils/
│       ├── config.ts              # Environment config
│       └── logger.ts              # Logging utility
└── tests/
    ├── unit/
    │   ├── agent/                 # Agent core tests
    │   ├── providers/             # Provider adapter tests
    │   └── services/              # Service tests
    ├── integration/
    │   └── api/                   # API endpoint tests
    └── fixtures/
        └── conversations.json     # Test conversation data

frontend/
├── src/
│   ├── components/
│   │   ├── ChatInterface.tsx      # Main chat UI
│   │   ├── MessageList.tsx        # Message display
│   │   └── InputBox.tsx           # User input
│   ├── services/
│   │   └── chatApi.ts             # API client
│   └── App.tsx                    # Root component
└── tests/
    └── components/                # Component tests
```

**Structure Decision**: Web application structure selected because the feature requires both a backend API (agent runtime, Cohere integration) and a frontend UI (chat interface). Backend handles agent logic, prompt orchestration, and Cohere API calls. Frontend provides user interaction layer.

## Complexity Tracking

> **No violations detected. This section is empty.**

All constitution requirements are satisfied without introducing unnecessary complexity. The architecture follows standard patterns:
- Agent abstraction is necessary for provider independence (constitution requirement)
- Prompt orchestration is standard for LLM applications
- Memory manager is optional and lightweight (session-based only)
- No over-engineering detected

---

## Phase 0: Research & Architecture Validation

**Objective**: Resolve all technical unknowns and validate architectural decisions before implementation.

### Research Tasks

#### R1: Cohere API Integration Patterns
**Question**: What are the best practices for integrating Cohere's chat/generate APIs in a Node.js agent architecture?

**Research Areas**:
- Cohere SDK initialization and configuration
- `cohere.chat()` vs `cohere.generate()` - which to use for conversational AI?
- Streaming vs non-streaming responses
- Error handling patterns (rate limits, timeouts, invalid keys)
- Token management and cost optimization

**Expected Output**:
- Code examples for Cohere SDK setup
- Recommended API methods for chat use case
- Error handling strategy
- Performance considerations

---

#### R2: OpenAI Agent SDK Pattern Adaptation
**Question**: How do we implement OpenAI Agent SDK patterns (Agent, Runner, Tool) without using OpenAI's actual SDK?

**Research Areas**:
- Core abstractions: Agent, Runner, Tool interfaces
- Execution flow: how Runner orchestrates Agent and Tools
- Tool calling mechanism compatible with Cohere
- State management during multi-turn conversations

**Expected Output**:
- TypeScript interface definitions for Agent, Runner, Tool
- Execution flow diagram
- Adapter pattern for Cohere integration

---

#### R3: Prompt Engineering for Safety Boundaries
**Question**: How do we design system prompts that reliably enforce safety boundaries (refuse medical/legal advice, no autonomous actions)?

**Research Areas**:
- System prompt templates for boundary enforcement
- Few-shot examples for refusal patterns
- Uncertainty admission phrasing
- Context injection strategies

**Expected Output**:
- System prompt templates
- Test cases for boundary enforcement
- Validation strategy

---

#### R4: Conversation Context Management
**Question**: What's the optimal strategy for maintaining conversation context in a stateless architecture?

**Research Areas**:
- Session-based memory patterns
- Context window management (token limits)
- Context summarization strategies
- State serialization for session storage

**Expected Output**:
- Context management architecture
- Session storage strategy
- Context pruning algorithm

---

#### R5: Tool Calling Simulation
**Question**: How do we implement tool-call-compatible outputs with Cohere for future extensibility?

**Research Areas**:
- Cohere's tool/function calling capabilities
- JSON schema for tool definitions
- Tool routing logic
- Response parsing and validation

**Expected Output**:
- Tool interface specification
- Tool calling flow diagram
- Example tool implementations

---

#### R6: Error Handling & Graceful Degradation
**Question**: What error scenarios must be handled, and what are appropriate fallback strategies?

**Research Areas**:
- Cohere API error types (rate limit, timeout, invalid key, model unavailable)
- Network failure handling
- Partial response handling
- User-friendly error messages

**Expected Output**:
- Error taxonomy
- Fallback strategies for each error type
- Error message templates

---

### Research Consolidation

**Output File**: `research.md`

**Format**:
```markdown
# Research Findings: AI Chatbot Assistant

## R1: Cohere API Integration Patterns
**Decision**: Use `cohere.chat()` for conversational interface
**Rationale**: [findings]
**Alternatives Considered**: [other options]
**Implementation Notes**: [code examples]

## R2: OpenAI Agent SDK Pattern Adaptation
**Decision**: [chosen approach]
**Rationale**: [findings]
...

[Continue for all research tasks]
```

---

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete with all decisions documented

### 1.1 Data Model Design

**Output File**: `data-model.md`

**Entities** (from spec.md Key Entities section):

#### Conversation
```typescript
interface Conversation {
  id: string;                    // Unique session identifier
  createdAt: Date;               // Session start time
  updatedAt: Date;               // Last activity time
  messages: Message[];           // Ordered message history
  context: Context;              // Accumulated context
  status: 'active' | 'ended';    // Session status
}
```

**Validation Rules**:
- `id` must be UUID v4
- `messages` array ordered chronologically
- `context` updated after each message
- Sessions expire after 30 minutes of inactivity

**State Transitions**:
- `active` → `ended` (user ends session or timeout)

---

#### Message
```typescript
interface Message {
  id: string;                    // Unique message identifier
  conversationId: string;        // Parent conversation
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text
  timestamp: Date;               // Message creation time
  metadata?: {
    model?: string;              // Cohere model used (for assistant)
    tokens?: number;             // Token count
    latency?: number;            // Response time (ms)
  };
}
```

**Validation Rules**:
- `content` must not be empty
- `role` must be 'user' or 'assistant'
- `conversationId` must reference valid Conversation

---

#### Context
```typescript
interface Context {
  conversationId: string;        // Parent conversation
  summary: string;               // Accumulated context summary
  topics: string[];              // Identified topics
  lastUpdated: Date;             // Last context update
  tokenCount: number;            // Approximate token usage
}
```

**Validation Rules**:
- `tokenCount` tracked to manage context window
- `summary` updated after each exchange
- `topics` extracted from conversation flow

---

#### ResponseFormat
```typescript
type ResponseFormat =
  | 'plain'                      // Plain text
  | 'structured'                 // Headers, lists, sections
  | 'code'                       // Code blocks with syntax
  | 'table';                     // Tabular data

interface FormattedResponse {
  content: string;               // Raw response content
  format: ResponseFormat;        // Detected format
  metadata: {
    hasCodeBlocks: boolean;
    hasLists: boolean;
    hasHeaders: boolean;
  };
}
```

---

### 1.2 API Contracts

**Output Directory**: `contracts/`

#### File: `chat-api.yaml` (OpenAPI 3.0)

```yaml
openapi: 3.0.0
info:
  title: AI Chatbot Assistant API
  version: 1.0.0
  description: Conversational AI assistant API with Cohere backend

paths:
  /api/chat/conversations:
    post:
      summary: Create new conversation
      responses:
        '201':
          description: Conversation created
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Conversation'
        '500':
          description: Server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Error'

  /api/chat/conversations/{conversationId}/messages:
    post:
      summary: Send message to conversation
      parameters:
        - name: conversationId
          in: path
          required: true
          schema:
            type: string
            format: uuid
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              properties:
                content:
                  type: string
                  description: User message content
              required:
                - content
      responses:
        '200':
          description: Assistant response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Message'
        '400':
          description: Invalid request
        '404':
          description: Conversation not found
        '500':
          description: Server error

  /api/chat/conversations/{conversationId}:
    get:
      summary: Get conversation history
      parameters:
        - name: conversationId
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        '200':
          description: Conversation details
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/Conversation'
        '404':
          description: Conversation not found

    delete:
      summary: End conversation
      parameters:
        - name: conversationId
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        '204':
          description: Conversation ended
        '404':
          description: Conversation not found

components:
  schemas:
    Conversation:
      type: object
      properties:
        id:
          type: string
          format: uuid
        createdAt:
          type: string
          format: date-time
        updatedAt:
          type: string
          format: date-time
        messages:
          type: array
          items:
            $ref: '#/components/schemas/Message'
        status:
          type: string
          enum: [active, ended]

    Message:
      type: object
      properties:
        id:
          type: string
          format: uuid
        conversationId:
          type: string
          format: uuid
        role:
          type: string
          enum: [user, assistant]
        content:
          type: string
        timestamp:
          type: string
          format: date-time
        metadata:
          type: object
          properties:
            model:
              type: string
            tokens:
              type: integer
            latency:
              type: integer

    Error:
      type: object
      properties:
        error:
          type: string
        message:
          type: string
        code:
          type: string
```

---

#### File: `agent-interface.ts` (TypeScript Interfaces)

```typescript
/**
 * Agent abstraction interface (OpenAI Agent SDK pattern)
 */
export interface IAgent {
  /**
   * Process user input and generate response
   */
  run(input: string, context: IContext): Promise<AgentResponse>;

  /**
   * Get agent configuration
   */
  getConfig(): AgentConfig;
}

/**
 * Provider abstraction interface
 */
export interface IProvider {
  /**
   * Generate completion from prompt
   */
  complete(prompt: string, options: CompletionOptions): Promise<CompletionResult>;

  /**
   * Chat-style completion with message history
   */
  chat(messages: ChatMessage[], options: CompletionOptions): Promise<CompletionResult>;

  /**
   * Check provider health
   */
  healthCheck(): Promise<boolean>;
}

/**
 * Tool abstraction interface (for future extensibility)
 */
export interface ITool {
  name: string;
  description: string;
  parameters: ToolParameters;
  execute(args: Record<string, any>): Promise<ToolResult>;
}

/**
 * Context interface for conversation state
 */
export interface IContext {
  conversationId: string;
  messages: ChatMessage[];
  summary: string;
  topics: string[];
}

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
  };
}

/**
 * Chat message structure
 */
export interface ChatMessage {
  role: 'system' | 'user' | 'assistant';
  content: string;
}

/**
 * Completion options
 */
export interface CompletionOptions {
  temperature?: number;
  maxTokens?: number;
  stopSequences?: string[];
  stream?: boolean;
}

/**
 * Completion result
 */
export interface CompletionResult {
  text: string;
  finishReason: 'complete' | 'length' | 'stop';
  usage: {
    promptTokens: number;
    completionTokens: number;
    totalTokens: number;
  };
}

/**
 * Tool call structure
 */
export interface ToolCall {
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
}

/**
 * Tool parameters schema
 */
export interface ToolParameters {
  type: 'object';
  properties: Record<string, ParameterSchema>;
  required: string[];
}

export interface ParameterSchema {
  type: string;
  description: string;
  enum?: string[];
}

export type ResponseFormat = 'plain' | 'structured' | 'code' | 'table';
```

---

### 1.3 Quickstart Guide

**Output File**: `quickstart.md`

```markdown
# AI Chatbot Assistant - Quickstart Guide

## Prerequisites

- Node.js 20.x or higher
- npm or yarn
- Cohere API key ([Get one here](https://cohere.com))

## Setup

### 1. Environment Configuration

Create `.env` file in the backend directory:

\`\`\`bash
COHERE_API_KEY=your_cohere_api_key_here
PORT=3000
NODE_ENV=development
\`\`\`

**IMPORTANT**: Never commit `.env` to version control. Add it to `.gitignore`.

### 2. Install Dependencies

\`\`\`bash
cd backend
npm install
\`\`\`

### 3. Run Tests

\`\`\`bash
npm test
\`\`\`

### 4. Start Development Server

\`\`\`bash
npm run dev
\`\`\`

Server will start at `http://localhost:3000`

## Usage

### Create Conversation

\`\`\`bash
curl -X POST http://localhost:3000/api/chat/conversations
\`\`\`

Response:
\`\`\`json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "createdAt": "2026-02-10T12:00:00Z",
  "status": "active",
  "messages": []
}
\`\`\`

### Send Message

\`\`\`bash
curl -X POST http://localhost:3000/api/chat/conversations/{conversationId}/messages \
  -H "Content-Type: application/json" \
  -d '{"content": "How do I implement error handling in Python?"}'
\`\`\`

Response:
\`\`\`json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "conversationId": "550e8400-e29b-41d4-a716-446655440000",
  "role": "assistant",
  "content": "Here's how to implement error handling in Python:\n\n1. Use try-except blocks...",
  "timestamp": "2026-02-10T12:00:05Z",
  "metadata": {
    "model": "command-r",
    "tokens": 150,
    "latency": 1200
  }
}
\`\`\`

### Get Conversation History

\`\`\`bash
curl http://localhost:3000/api/chat/conversations/{conversationId}
\`\`\`

### End Conversation

\`\`\`bash
curl -X DELETE http://localhost:3000/api/chat/conversations/{conversationId}
\`\`\`

## Testing

### Unit Tests

\`\`\`bash
npm run test:unit
\`\`\`

### Integration Tests

\`\`\`bash
npm run test:integration
\`\`\`

### Manual Testing

Use the provided test script:

\`\`\`bash
npm run test:conversation
\`\`\`

This will run a sample conversation flow and validate responses.

## Troubleshooting

### "Invalid API Key" Error

- Verify `COHERE_API_KEY` is set in `.env`
- Check API key is valid at [Cohere Dashboard](https://dashboard.cohere.com)

### "Rate Limit Exceeded" Error

- Cohere free tier has rate limits
- Implement exponential backoff (already included in error handling)
- Consider upgrading Cohere plan

### Slow Response Times

- Check network latency to Cohere API
- Consider using streaming responses for better UX
- Monitor token usage (large contexts increase latency)

## Next Steps

- Integrate frontend UI (see `frontend/` directory)
- Implement tool calling for extensibility
- Add conversation persistence (database)
- Deploy to production environment

## Architecture Overview

\`\`\`
User Input
    ↓
System Prompt Injection
    ↓
Agent Reasoning (Agent.run)
    ↓
Cohere Completion (CohereAdapter.chat)
    ↓
Tool Routing (if applicable)
    ↓
Final User Response
\`\`\`

For detailed architecture, see [plan.md](./plan.md).
```

---

### 1.4 Agent Context Update

**Action**: Run agent context update script

```bash
powershell.exe -ExecutionPolicy Bypass -File ".specify/scripts/powershell/update-agent-context.ps1" -AgentType claude
```

This will update the Claude-specific context file with:
- Cohere API integration details
- TypeScript/Node.js stack information
- Agent architecture patterns
- Project structure

---

## Phase 2: Task Generation

**NOT PERFORMED BY THIS COMMAND**

After Phase 1 completes, run `/sp.tasks` to generate actionable, dependency-ordered tasks in `tasks.md`.

---

## Constitution Re-Check (Post-Design)

### Technical Accuracy ✅
- Data model entities match spec requirements
- API contracts follow REST best practices
- No fabricated capabilities

### AI Assistant Integration ✅
- Provider abstraction implemented via `IProvider` interface
- Cohere adapter isolated in `CohereAdapter.ts`
- API key via environment variable enforced
- Error handling with graceful fallback designed
- Tool calling interface defined for future extensibility

### Professional Quality ✅
- OpenAPI specification for API contracts
- TypeScript interfaces for type safety
- Comprehensive quickstart guide
- Testing strategy defined

**FINAL GATE RESULT**: ✅ **PASS** - Design maintains constitution compliance.

---

## Summary

**Deliverables**:
1. ✅ `plan.md` - This implementation plan
2. 🔄 `research.md` - To be generated in Phase 0
3. 🔄 `data-model.md` - To be generated in Phase 1
4. 🔄 `contracts/` - To be generated in Phase 1
5. 🔄 `quickstart.md` - To be generated in Phase 1

**Next Command**: This plan stops here. Research and design artifacts will be generated as part of the planning workflow execution.

**Ready for**: `/sp.tasks` after Phase 1 completion
