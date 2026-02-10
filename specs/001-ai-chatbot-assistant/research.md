# Research Findings: AI Chatbot Assistant

**Date**: 2026-02-10
**Feature**: 001-ai-chatbot-assistant
**Purpose**: Resolve technical unknowns and validate architectural decisions

---

## R1: Cohere API Integration Patterns

**Decision**: Use `cohere.chat()` for conversational interface with message history support

**Rationale**:
- `cohere.chat()` is purpose-built for conversational AI with native support for message history
- Handles system prompts, user messages, and assistant responses in a structured format
- Better context management compared to `cohere.generate()` which is designed for single completions
- Supports streaming responses for improved UX
- Built-in conversation memory handling

**Alternatives Considered**:
- `cohere.generate()`: Rejected because it requires manual message history formatting and lacks native conversation support
- Custom prompt assembly with generate(): More complex, error-prone, and doesn't leverage Cohere's conversation optimizations

**Implementation Notes**:

```typescript
import { CohereClient } from 'cohere-ai';

// Initialize client
const cohere = new CohereClient({
  token: process.env.COHERE_API_KEY,
});

// Chat completion
const response = await cohere.chat({
  model: 'command-r', // or 'command-r-plus' for higher quality
  message: userInput,
  chatHistory: previousMessages.map(msg => ({
    role: msg.role,
    message: msg.content,
  })),
  preamble: systemPrompt, // System instructions
  temperature: 0.7,
  maxTokens: 1000,
});

// Access response
const assistantMessage = response.text;
const tokenUsage = response.meta?.tokens;
```

**Error Handling Strategy**:
- Wrap all Cohere calls in try-catch blocks
- Handle specific error types: `CohereAPIError`, `CohereTimeoutError`, `CohereRateLimitError`
- Implement exponential backoff for rate limits (start with 1s, max 32s)
- Graceful degradation: return user-friendly error message on failure

**Token Management**:
- Track token usage per request via `response.meta.tokens`
- Implement context window management (Command-R: 128k tokens, Command-R+: 128k tokens)
- Prune old messages when approaching token limits
- Estimate tokens before API call: ~4 characters per token (rough approximation)

**Performance Considerations**:
- Non-streaming: 1-3s typical latency for short responses
- Streaming: First token in ~500ms, improves perceived performance
- Use streaming for responses >200 tokens
- Cache system prompts to reduce token usage

---

## R2: OpenAI Agent SDK Pattern Adaptation

**Decision**: Implement Agent, Runner, and Tool abstractions as TypeScript interfaces without OpenAI SDK dependency

**Rationale**:
- OpenAI Agent SDK patterns provide proven architecture for LLM agents
- Abstractions enable provider independence (can swap Cohere for OpenAI, Anthropic, etc.)
- Clear separation of concerns: Agent (logic), Runner (execution), Tool (capabilities)
- Facilitates testing and mocking

**Architecture**:

```
┌─────────────────────────────────────────────────┐
│                    Runner                        │
│  - Orchestrates execution flow                   │
│  - Manages conversation state                    │
│  - Handles tool routing                          │
└─────────────┬───────────────────────────────────┘
              │
              ├──────────────┐
              │              │
    ┌─────────▼────────┐   ┌▼──────────────┐
    │      Agent       │   │     Tools     │
    │  - Process input │   │  - Execute    │
    │  - Generate resp │   │    functions  │
    │  - Use provider  │   │  - Return     │
    └─────────┬────────┘   │    results    │
              │            └───────────────┘
    ┌─────────▼────────┐
    │  CohereAdapter   │
    │  - Implements    │
    │    IProvider     │
    │  - Calls Cohere  │
    │    API           │
    └──────────────────┘
```

**Core Interfaces** (see `contracts/agent-interface.ts` in plan.md):
- `IAgent`: Main agent interface with `run()` method
- `IProvider`: LLM provider abstraction with `chat()` and `complete()` methods
- `ITool`: Tool interface for future extensibility
- `IContext`: Conversation context management

**Execution Flow**:
1. User input received by Runner
2. Runner loads conversation context
3. Runner calls Agent.run(input, context)
4. Agent assembles prompt with system instructions
5. Agent calls Provider.chat(messages)
6. Provider (CohereAdapter) makes Cohere API call
7. If tool calls detected, Runner routes to appropriate Tool
8. Tool executes and returns result
9. Agent incorporates tool result into response
10. Runner returns final response to user

**State Management**:
- Runner maintains conversation state in memory (Map<conversationId, Conversation>)
- Context updated after each exchange
- Session timeout: 30 minutes of inactivity
- No persistent storage in MVP (can add later)

**Implementation Example**:

```typescript
class ChatAgent implements IAgent {
  constructor(private provider: IProvider, private config: AgentConfig) {}

  async run(input: string, context: IContext): Promise<AgentResponse> {
    // Assemble messages
    const messages: ChatMessage[] = [
      { role: 'system', content: this.config.systemPrompt },
      ...context.messages,
      { role: 'user', content: input },
    ];

    // Call provider
    const result = await this.provider.chat(messages, {
      temperature: this.config.temperature,
      maxTokens: this.config.maxTokens,
    });

    // Return formatted response
    return {
      content: result.text,
      format: this.detectFormat(result.text),
      metadata: {
        model: this.config.model,
        tokens: result.usage.totalTokens,
        latency: Date.now() - startTime,
      },
    };
  }
}
```

---

## R3: Prompt Engineering for Safety Boundaries

**Decision**: Use multi-layered system prompts with explicit refusal patterns and uncertainty admission

**Rationale**:
- System prompts are the primary mechanism for enforcing safety boundaries
- Explicit instructions reduce hallucination and inappropriate responses
- Few-shot examples improve refusal consistency
- Layered approach: general guidelines + specific refusal patterns + uncertainty handling

**System Prompt Template**:

```typescript
const SYSTEM_PROMPT = `You are a professional AI assistant designed to help developers, students, and general users with technical questions and guidance.

## Core Capabilities
- Answer technical questions about programming, software development, and computer science
- Provide step-by-step explanations of complex concepts
- Offer code examples and best practices
- Maintain context across multi-turn conversations

## Safety Boundaries (CRITICAL - MUST FOLLOW)

### Topics You MUST Refuse
1. Medical diagnosis or treatment advice
   - Response: "I cannot provide medical advice. Please consult a qualified healthcare professional."

2. Legal advice or interpretation
   - Response: "I cannot provide legal advice. Please consult a qualified attorney."

3. Financial investment advice
   - Response: "I cannot provide financial advice. Please consult a qualified financial advisor."

4. Autonomous actions without explicit user approval
   - Response: "I cannot perform autonomous actions. I can guide you through the steps, but you must execute them."

### Uncertainty Handling (CRITICAL - MUST FOLLOW)
- If you are uncertain about information, explicitly state: "I'm not certain about this, but based on my knowledge..."
- If information is outside your knowledge cutoff, state: "My knowledge is current as of [date]. For the latest information, please check [authoritative source]."
- Never fabricate facts, citations, or code examples. If you don't know, say so.

## Response Style
- Professional and neutral tone
- Clear, structured formatting (use headers, lists, code blocks)
- Concise but complete explanations
- Admit limitations when appropriate

## Examples

User: "Can you diagnose my headache?"
Assistant: "I cannot provide medical advice. Headaches can have many causes, and proper diagnosis requires a healthcare professional. Please consult a doctor if you're concerned."

User: "Automatically fix all bugs in my codebase"
Assistant: "I cannot perform autonomous actions on your codebase. However, I can help you identify common bug patterns and guide you through debugging steps. Would you like me to explain debugging strategies?"

User: "What's the latest version of React?"
Assistant: "My knowledge is current as of January 2025. At that time, React 18 was the latest stable version. For the most current version, please check the official React documentation at react.dev."
`;
```

**Few-Shot Examples for Refusal**:
- Include 3-5 examples of proper refusal in system prompt
- Cover each boundary category (medical, legal, financial, autonomous)
- Demonstrate polite but firm refusal with helpful redirection

**Uncertainty Admission Phrasing**:
- "I'm not certain, but..."
- "Based on my knowledge as of [date]..."
- "I don't have enough information to answer definitively..."
- "This is outside my area of expertise..."

**Context Injection Strategy**:
- System prompt injected at the start of every conversation
- Reinforced with each API call (Cohere's `preamble` parameter)
- No user-controllable system prompt modification
- Validate responses post-generation for boundary violations (safety layer)

**Validation Strategy**:
- Test suite with 50+ boundary test cases
- Automated testing: submit prohibited questions, verify refusal
- Manual review: check tone and helpfulness of refusals
- Red team testing: adversarial prompts attempting to bypass boundaries

---

## R4: Conversation Context Management

**Decision**: Session-based in-memory context with sliding window and token-aware pruning

**Rationale**:
- Stateless architecture (no database) simplifies MVP
- In-memory storage sufficient for 100 concurrent conversations
- Sliding window maintains recent context while managing token limits
- Token-aware pruning prevents context overflow

**Context Management Architecture**:

```typescript
class ConversationMemory {
  private conversations: Map<string, Conversation> = new Map();
  private readonly MAX_CONTEXT_TOKENS = 100000; // Leave room for response
  private readonly SESSION_TIMEOUT_MS = 30 * 60 * 1000; // 30 minutes

  addMessage(conversationId: string, message: Message): void {
    const conversation = this.conversations.get(conversationId);
    if (!conversation) throw new Error('Conversation not found');

    conversation.messages.push(message);
    conversation.updatedAt = new Date();

    // Update context
    this.updateContext(conversation);

    // Prune if necessary
    if (this.estimateTokens(conversation) > this.MAX_CONTEXT_TOKENS) {
      this.pruneOldMessages(conversation);
    }
  }

  private updateContext(conversation: Conversation): void {
    // Extract topics from recent messages
    const recentMessages = conversation.messages.slice(-10);
    const topics = this.extractTopics(recentMessages);

    // Generate summary of older messages
    const summary = this.summarizeOldMessages(conversation.messages);

    conversation.context = {
      conversationId: conversation.id,
      summary,
      topics,
      lastUpdated: new Date(),
      tokenCount: this.estimateTokens(conversation),
    };
  }

  private pruneOldMessages(conversation: Conversation): void {
    // Keep system prompt + last 20 messages
    const systemMessages = conversation.messages.filter(m => m.role === 'system');
    const recentMessages = conversation.messages.slice(-20);

    // Summarize pruned messages
    const prunedMessages = conversation.messages.slice(0, -20);
    const summary = this.summarizeMessages(prunedMessages);

    // Update conversation
    conversation.messages = [
      ...systemMessages,
      { role: 'system', content: `Previous conversation summary: ${summary}` },
      ...recentMessages,
    ];
  }

  private estimateTokens(conversation: Conversation): number {
    // Rough estimate: 4 characters per token
    const totalChars = conversation.messages.reduce(
      (sum, msg) => sum + msg.content.length,
      0
    );
    return Math.ceil(totalChars / 4);
  }
}
```

**Session Storage Strategy**:
- In-memory Map for MVP (conversationId → Conversation)
- Session expiration: 30 minutes of inactivity
- Cleanup job runs every 5 minutes to remove expired sessions
- Future: Redis for distributed sessions, PostgreSQL for persistence

**Context Pruning Algorithm**:
1. Estimate total token count for conversation
2. If > 100k tokens (leaving 28k for response):
   - Keep system prompt
   - Keep last 20 messages (recent context)
   - Summarize older messages into single context message
   - Insert summary as system message
3. Update context metadata

**Context Summarization**:
- Use Cohere to generate summary of pruned messages
- Summary format: "Previous conversation covered: [topics]. Key points: [points]."
- Cache summaries to avoid repeated API calls
- Max summary length: 500 tokens

**State Serialization** (for future persistence):
```typescript
interface SerializedConversation {
  id: string;
  createdAt: string; // ISO 8601
  updatedAt: string;
  messages: SerializedMessage[];
  context: SerializedContext;
  status: 'active' | 'ended';
}

// JSON serialization for Redis/DB storage
const serialized = JSON.stringify(conversation);
const deserialized = JSON.parse(serialized) as SerializedConversation;
```

---

## R5: Tool Calling Simulation

**Decision**: Implement tool interface with JSON schema definitions, compatible with Cohere's tool calling capabilities

**Rationale**:
- Cohere supports tool/function calling via structured outputs
- Tool interface enables future extensibility (web search, code execution, etc.)
- JSON schema provides type safety and validation
- Routing logic separates tool execution from agent logic

**Cohere Tool Calling Capabilities**:
- Cohere Command-R and Command-R+ support tool calling
- Tools defined via JSON schema (similar to OpenAI function calling)
- Model decides when to call tools based on user input
- Tool results fed back into conversation for final response

**Tool Interface Specification** (see `contracts/agent-interface.ts`):

```typescript
interface ITool {
  name: string;
  description: string;
  parameters: ToolParameters;
  execute(args: Record<string, any>): Promise<ToolResult>;
}

// Example tool: Calculator
const calculatorTool: ITool = {
  name: 'calculator',
  description: 'Performs basic arithmetic operations',
  parameters: {
    type: 'object',
    properties: {
      operation: {
        type: 'string',
        description: 'The operation to perform',
        enum: ['add', 'subtract', 'multiply', 'divide'],
      },
      a: {
        type: 'number',
        description: 'First operand',
      },
      b: {
        type: 'number',
        description: 'Second operand',
      },
    },
    required: ['operation', 'a', 'b'],
  },
  async execute(args) {
    const { operation, a, b } = args;
    let result: number;

    switch (operation) {
      case 'add': result = a + b; break;
      case 'subtract': result = a - b; break;
      case 'multiply': result = a * b; break;
      case 'divide':
        if (b === 0) return { success: false, error: 'Division by zero' };
        result = a / b;
        break;
      default:
        return { success: false, error: 'Invalid operation' };
    }

    return { success: true, data: result };
  },
};
```

**Tool Calling Flow**:

```
User: "What's 15 * 23?"
    ↓
Agent receives input
    ↓
Cohere API call with tools defined
    ↓
Cohere responds: "I'll use the calculator tool"
Tool call: { name: 'calculator', args: { operation: 'multiply', a: 15, b: 23 } }
    ↓
Runner routes to calculator tool
    ↓
Tool executes: 15 * 23 = 345
    ↓
Tool result fed back to Cohere
    ↓
Cohere generates final response: "15 multiplied by 23 equals 345."
    ↓
Response returned to user
```

**Implementation Example**:

```typescript
// Register tools with Cohere
const response = await cohere.chat({
  message: userInput,
  tools: [
    {
      name: calculatorTool.name,
      description: calculatorTool.description,
      parameterDefinitions: calculatorTool.parameters.properties,
    },
  ],
  chatHistory: previousMessages,
});

// Check if tool calls were made
if (response.toolCalls && response.toolCalls.length > 0) {
  const toolResults = await Promise.all(
    response.toolCalls.map(async (call) => {
      const tool = this.tools.get(call.name);
      if (!tool) throw new Error(`Tool not found: ${call.name}`);

      const result = await tool.execute(call.parameters);
      return {
        call: call.name,
        outputs: [result],
      };
    })
  );

  // Feed tool results back to Cohere
  const finalResponse = await cohere.chat({
    message: userInput,
    tools: [...], // Same tools
    toolResults,
    chatHistory: [...previousMessages, response],
  });

  return finalResponse.text;
}
```

**Response Parsing and Validation**:
- Validate tool call arguments against JSON schema
- Handle tool execution errors gracefully
- Timeout tool execution after 5 seconds
- Log tool calls for debugging and monitoring

**Example Tool Implementations** (for future):
- Web search tool (integrate with search API)
- Code execution tool (sandboxed Python/JavaScript runner)
- File system tool (read/write files with permissions)
- Database query tool (execute SQL queries)

---

## R6: Error Handling & Graceful Degradation

**Decision**: Comprehensive error taxonomy with specific fallback strategies for each error type

**Rationale**:
- Cohere API can fail for multiple reasons (rate limits, timeouts, invalid keys, model unavailable)
- Users need clear, actionable error messages
- System should degrade gracefully without crashing
- Retry logic for transient errors, fail fast for permanent errors

**Error Taxonomy**:

| Error Type | Cause | Fallback Strategy | User Message |
|------------|-------|-------------------|--------------|
| `InvalidAPIKey` | Wrong/missing API key | Fail fast, log error | "Service configuration error. Please contact support." |
| `RateLimitExceeded` | Too many requests | Exponential backoff, retry | "Service is busy. Retrying..." |
| `Timeout` | Network/API slow | Retry once, then fail | "Request timed out. Please try again." |
| `ModelUnavailable` | Cohere model down | Fail fast | "AI service temporarily unavailable. Please try again later." |
| `InvalidRequest` | Malformed input | Validate input, return error | "Invalid input. Please check your message." |
| `ContextTooLong` | Token limit exceeded | Prune context, retry | "Conversation too long. Starting fresh context." |
| `NetworkError` | Connection failed | Retry once | "Network error. Please check your connection." |
| `UnknownError` | Unexpected error | Log, fail gracefully | "An unexpected error occurred. Please try again." |

**Fallback Strategies**:

1. **Rate Limit Handling**:
```typescript
async function callCohereWithRetry(
  fn: () => Promise<any>,
  maxRetries = 3
): Promise<any> {
  let delay = 1000; // Start with 1 second

  for (let attempt = 0; attempt < maxRetries; attempt++) {
    try {
      return await fn();
    } catch (error) {
      if (error instanceof CohereRateLimitError && attempt < maxRetries - 1) {
        await sleep(delay);
        delay *= 2; // Exponential backoff
        continue;
      }
      throw error;
    }
  }
}
```

2. **Timeout Handling**:
```typescript
async function callWithTimeout<T>(
  promise: Promise<T>,
  timeoutMs: number
): Promise<T> {
  const timeoutPromise = new Promise<never>((_, reject) =>
    setTimeout(() => reject(new Error('Timeout')), timeoutMs)
  );
  return Promise.race([promise, timeoutPromise]);
}

// Usage
const response = await callWithTimeout(
  cohere.chat({ ... }),
  10000 // 10 second timeout
);
```

3. **Context Overflow Handling**:
```typescript
try {
  const response = await cohere.chat({ ... });
} catch (error) {
  if (error.message.includes('context_length_exceeded')) {
    // Prune context and retry
    this.pruneContext(conversation);
    const response = await cohere.chat({ ... });
    return response;
  }
  throw error;
}
```

4. **Partial Response Handling**:
```typescript
// For streaming responses
const chunks: string[] = [];
try {
  for await (const chunk of stream) {
    chunks.push(chunk.text);
  }
} catch (error) {
  // Return partial response if available
  if (chunks.length > 0) {
    return {
      content: chunks.join(''),
      partial: true,
      error: 'Response incomplete due to error',
    };
  }
  throw error;
}
```

**User-Friendly Error Messages**:

```typescript
function getUserFriendlyError(error: Error): string {
  if (error instanceof CohereAPIError) {
    switch (error.statusCode) {
      case 401:
        return 'Service configuration error. Please contact support.';
      case 429:
        return 'Service is busy. Please wait a moment and try again.';
      case 500:
      case 503:
        return 'AI service temporarily unavailable. Please try again in a few minutes.';
      default:
        return 'An error occurred while processing your request. Please try again.';
    }
  }

  if (error.message.includes('timeout')) {
    return 'Request timed out. Please try again with a shorter message.';
  }

  if (error.message.includes('network')) {
    return 'Network error. Please check your internet connection.';
  }

  // Generic fallback
  return 'An unexpected error occurred. Please try again later.';
}
```

**Error Logging**:
- Log all errors with context (conversationId, userId, timestamp, error details)
- Use structured logging (JSON format)
- Include stack traces for debugging
- Monitor error rates and alert on spikes

**Graceful Degradation Examples**:
- If Cohere API is down: Display maintenance message, queue requests for retry
- If context too long: Automatically prune and retry transparently
- If rate limited: Show "Service busy" message with retry countdown
- If network error: Retry once automatically, then prompt user to retry manually

---

## Summary

All research tasks completed. Key decisions:

1. **Cohere Integration**: Use `cohere.chat()` with message history support
2. **Agent Architecture**: OpenAI SDK patterns (Agent, Runner, Tool) without OpenAI dependency
3. **Safety Boundaries**: Multi-layered system prompts with explicit refusal patterns
4. **Context Management**: Session-based in-memory with token-aware pruning
5. **Tool Calling**: JSON schema-based tool interface compatible with Cohere
6. **Error Handling**: Comprehensive error taxonomy with specific fallback strategies

**No unresolved NEEDS CLARIFICATION items remain.**

**Ready for Phase 1**: Design artifacts (data-model.md, contracts/, quickstart.md)
