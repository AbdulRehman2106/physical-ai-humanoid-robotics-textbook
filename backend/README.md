# AI Chatbot Assistant

A professional, context-aware AI chatbot powered by Cohere, built with TypeScript and following OpenAI Agent SDK patterns.

## Features

✅ **Technical Q&A** - Accurate responses to programming and technical questions
✅ **Step-by-Step Guidance** - Structured explanations with numbered steps
✅ **Context Maintenance** - Multi-turn conversations with context awareness
✅ **Safety Boundaries** - Refuses medical/legal/financial advice and autonomous actions
✅ **Professional Formatting** - Automatic detection and use of headers, lists, code blocks, tables
✅ **Token Management** - Automatic context pruning when token limits exceeded
✅ **Error Handling** - Graceful degradation with user-friendly error messages

## Architecture

```
User Input
    ↓
Safety Pre-Processing (detect prohibited topics)
    ↓
System Prompt Injection (safety boundaries + formatting guidelines)
    ↓
Agent Reasoning (ChatAgent.run)
    ↓
Cohere Completion (CohereAdapter.chat)
    ↓
Format Detection (plain, structured, code, table)
    ↓
Safety Post-Processing (validate response)
    ↓
Final User Response
```

## Tech Stack

- **Runtime**: Node.js 20.x + TypeScript 5.x
- **LLM Provider**: Cohere (Command-R)
- **API Framework**: Express.js
- **Validation**: Zod
- **Testing**: Jest + Supertest
- **Architecture**: OpenAI Agent SDK patterns (provider-agnostic)

## Quick Start

### Prerequisites

- Node.js 20.x or higher
- Cohere API key ([Get one here](https://dashboard.cohere.com/api-keys))

### Installation

```bash
# Clone the repository
cd backend

# Install dependencies
npm install

# Configure environment
cp .env.example .env
# Edit .env and add your COHERE_API_KEY
```

### Configuration

Edit `backend/.env`:

```bash
COHERE_API_KEY=your_cohere_api_key_here
PORT=3000
NODE_ENV=development
LOG_LEVEL=info
```

### Run

```bash
# Development mode (with hot reload)
npm run dev

# Production mode
npm run build
npm start

# Run tests
npm test
```

## API Endpoints

### Create Conversation

```bash
POST /api/chat/conversations

Response:
{
  "id": "uuid",
  "createdAt": "2026-02-10T12:00:00Z",
  "status": "active",
  "messages": []
}
```

### Send Message

```bash
POST /api/chat/conversations/:conversationId/messages
Content-Type: application/json

{
  "content": "How do I implement error handling in Python?"
}

Response:
{
  "id": "msg_...",
  "conversationId": "uuid",
  "role": "assistant",
  "content": "Here's how to implement error handling...",
  "timestamp": "2026-02-10T12:00:05Z",
  "metadata": {
    "model": "command-r",
    "tokens": 150,
    "latency": 1200
  }
}
```

### Get Conversation History

```bash
GET /api/chat/conversations/:conversationId

Response:
{
  "id": "uuid",
  "createdAt": "2026-02-10T12:00:00Z",
  "updatedAt": "2026-02-10T12:05:00Z",
  "messages": [...],
  "context": {
    "summary": "...",
    "topics": ["Python", "error handling"],
    "tokenCount": 1250
  },
  "status": "active"
}
```

### End Conversation

```bash
DELETE /api/chat/conversations/:conversationId

Response: 204 No Content
```

### Health Check

```bash
GET /api/health

Response:
{
  "status": "healthy",
  "timestamp": "2026-02-10T12:00:00Z",
  "services": {
    "cohere": "available"
  }
}
```

## Project Structure

```
backend/
├── src/
│   ├── agent/
│   │   ├── core/           # Agent, Runner, Tool interfaces
│   │   ├── providers/      # CohereAdapter (provider abstraction)
│   │   ├── prompts/        # SystemPrompts, PromptOrchestrator
│   │   └── memory/         # ConversationMemory (session management)
│   ├── api/
│   │   ├── routes/         # Express routes (chat, health)
│   │   ├── middleware/     # Error handling, validation
│   │   └── server.ts       # Express app setup
│   ├── models/             # Conversation, Message, Context entities
│   ├── services/           # ChatService, SafetyService
│   └── utils/              # Config, logger, errors
└── tests/
    ├── unit/               # Unit tests
    ├── integration/        # API integration tests
    └── fixtures/           # Test data
```

## Safety Features

### Prohibited Topics

The chatbot automatically detects and refuses:
- **Medical advice** - "I cannot provide medical advice. Please consult a qualified healthcare professional."
- **Legal advice** - "I cannot provide legal advice. Please consult a qualified attorney."
- **Financial advice** - "I cannot provide financial advice. Please consult a qualified financial advisor."
- **Autonomous actions** - "I cannot perform autonomous actions. I can guide you through the steps, but you must execute them."

### Uncertainty Admission

The chatbot explicitly states when:
- Information is uncertain
- Knowledge is outdated (cutoff: January 2025)
- Topic is outside expertise

## Context Management

- **Session Timeout**: 30 minutes of inactivity
- **Token Limit**: 100,000 tokens (auto-pruning when exceeded)
- **Message Retention**: Last 20 messages kept, older messages summarized
- **Topic Tracking**: Automatically extracts and tracks conversation topics
- **Context Shift Detection**: Detects when conversation topic changes

## Error Handling

- **Rate Limiting**: Exponential backoff (1s → 2s → 4s)
- **Timeouts**: 10-second timeout for API calls
- **Graceful Degradation**: User-friendly error messages
- **Retry Logic**: Up to 3 retries for transient errors

## Development

### Run Tests

```bash
# All tests
npm test

# Unit tests only
npm run test:unit

# Integration tests only
npm run test:integration

# With coverage
npm run test:coverage
```

### Linting & Formatting

```bash
# Lint
npm run lint

# Format
npm run format
```

## Performance

- **Response Time**: <2s for 95% of queries
- **Concurrent Conversations**: Supports 100+ concurrent sessions
- **Token Efficiency**: Automatic context pruning to manage costs
- **Memory Usage**: In-memory storage (MVP), extensible to Redis/PostgreSQL

## Deployment

### Environment Variables

```bash
# Required
COHERE_API_KEY=your_key

# Optional
PORT=3000
NODE_ENV=production
LOG_LEVEL=info
SESSION_TIMEOUT_MS=1800000
MAX_CONTEXT_TOKENS=100000
ALLOWED_ORIGINS=https://yourdomain.com
```

### Docker (Optional)

```dockerfile
FROM node:20-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY . .
RUN npm run build
EXPOSE 3000
CMD ["npm", "start"]
```

## Troubleshooting

### "Invalid API Key" Error

- Verify `COHERE_API_KEY` is set in `.env`
- Check key is valid at [Cohere Dashboard](https://dashboard.cohere.com)

### "Rate Limit Exceeded" Error

- Cohere free tier: 100 requests/minute
- Automatic retry with exponential backoff
- Consider upgrading Cohere plan

### Slow Response Times

- Check network latency to Cohere API
- Monitor token usage (large contexts = slower)
- Consider using streaming responses (future enhancement)

## Roadmap

- [ ] Frontend UI (React/Next.js)
- [ ] Tool calling support (web search, code execution)
- [ ] Streaming responses
- [ ] Conversation persistence (PostgreSQL)
- [ ] User authentication
- [ ] Rate limiting middleware
- [ ] Deployment guides (Vercel, Railway, AWS)

## License

MIT

## Support

For issues or questions:
1. Check this README
2. Review API documentation
3. Check logs for error details
4. Open an issue on GitHub

---

**Built with ❤️ using Cohere and TypeScript**
