# AI Chatbot Assistant - Quickstart Guide

**Feature**: 001-ai-chatbot-assistant
**Version**: 1.0.0
**Last Updated**: 2026-02-10

---

## Prerequisites

- **Node.js**: 20.x or higher
- **npm** or **yarn**: Latest version
- **Cohere API Key**: [Get one here](https://dashboard.cohere.com/api-keys)
- **Git**: For version control

---

## Setup

### 1. Environment Configuration

Create a `.env` file in the `backend/` directory:

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Server Configuration
PORT=3000
NODE_ENV=development

# Optional: Logging
LOG_LEVEL=info
```

**⚠️ IMPORTANT SECURITY NOTES**:
- **NEVER** commit `.env` to version control
- Add `.env` to `.gitignore` immediately
- Use different API keys for development and production
- Rotate API keys regularly

**Get Your Cohere API Key**:
1. Visit [https://dashboard.cohere.com](https://dashboard.cohere.com)
2. Sign up or log in
3. Navigate to API Keys section
4. Create a new API key
5. Copy and paste into `.env` file

---

### 2. Install Dependencies

```bash
# Navigate to backend directory
cd backend

# Install dependencies
npm install

# Expected packages:
# - cohere-ai (Cohere SDK)
# - express or fastify (API server)
# - zod (validation)
# - uuid (ID generation)
# - dotenv (environment variables)
```

**Verify Installation**:
```bash
npm list cohere-ai
# Should show: cohere-ai@x.x.x
```

---

### 3. Run Tests

```bash
# Run all tests
npm test

# Run unit tests only
npm run test:unit

# Run integration tests only
npm run test:integration

# Run with coverage
npm run test:coverage
```

**Expected Output**:
```
✓ Agent core tests (15 tests)
✓ Provider adapter tests (10 tests)
✓ API endpoint tests (12 tests)
✓ Context management tests (8 tests)

Total: 45 tests passed
Coverage: 85%+
```

---

### 4. Start Development Server

```bash
# Start server with hot reload
npm run dev

# Or start production build
npm run build
npm start
```

**Expected Output**:
```
🚀 AI Chatbot Assistant API
📡 Server listening on http://localhost:3000
✅ Cohere provider initialized
✅ Health check passed
```

**Verify Server is Running**:
```bash
curl http://localhost:3000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2026-02-10T12:00:00Z",
  "services": {
    "cohere": "available"
  }
}
```

---

## Usage

### Basic Conversation Flow

#### 1. Create Conversation

```bash
curl -X POST http://localhost:3000/api/chat/conversations \
  -H "Content-Type: application/json"
```

**Response**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "createdAt": "2026-02-10T12:00:00Z",
  "updatedAt": "2026-02-10T12:00:00Z",
  "messages": [],
  "status": "active"
}
```

**Save the `id` for subsequent requests.**

---

#### 2. Send Message

```bash
curl -X POST http://localhost:3000/api/chat/conversations/550e8400-e29b-41d4-a716-446655440000/messages \
  -H "Content-Type: application/json" \
  -d '{
    "content": "How do I implement error handling in Python?"
  }'
```

**Response**:
```json
{
  "id": "660e8400-e29b-41d4-a716-446655440001",
  "conversationId": "550e8400-e29b-41d4-a716-446655440000",
  "role": "assistant",
  "content": "Here's how to implement error handling in Python:\n\n1. **Use try-except blocks**:\n```python\ntry:\n    result = risky_operation()\nexcept ValueError as e:\n    print(f\"Error: {e}\")\n```\n\n2. **Handle multiple exceptions**:\n```python\ntry:\n    result = operation()\nexcept (ValueError, TypeError) as e:\n    handle_error(e)\n```\n\n3. **Use finally for cleanup**:\n```python\ntry:\n    file = open('data.txt')\n    process(file)\nfinally:\n    file.close()\n```\n\nBest practices:\n- Catch specific exceptions, not generic `Exception`\n- Use context managers (`with` statement) for resources\n- Log errors for debugging\n- Provide user-friendly error messages",
  "timestamp": "2026-02-10T12:00:05Z",
  "metadata": {
    "model": "command-r",
    "tokens": 245,
    "latency": 1450
  }
}
```

---

#### 3. Continue Conversation (Context Maintained)

```bash
curl -X POST http://localhost:3000/api/chat/conversations/550e8400-e29b-41d4-a716-446655440000/messages \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Can you show me an example with file operations?"
  }'
```

**Response** (context-aware):
```json
{
  "id": "770e8400-e29b-41d4-a716-446655440002",
  "conversationId": "550e8400-e29b-41d4-a716-446655440000",
  "role": "assistant",
  "content": "Here's a complete example of error handling with file operations in Python:\n\n```python\ndef read_config_file(filename):\n    try:\n        with open(filename, 'r') as file:\n            data = file.read()\n            return data\n    except FileNotFoundError:\n        print(f\"Error: {filename} not found\")\n        return None\n    except PermissionError:\n        print(f\"Error: No permission to read {filename}\")\n        return None\n    except Exception as e:\n        print(f\"Unexpected error: {e}\")\n        return None\n\n# Usage\nconfig = read_config_file('config.txt')\nif config:\n    print(\"Config loaded successfully\")\nelse:\n    print(\"Using default configuration\")\n```\n\nThis example demonstrates:\n- Using `with` statement for automatic file closing\n- Catching specific file-related exceptions\n- Providing fallback behavior\n- Graceful error handling",
  "timestamp": "2026-02-10T12:00:12Z",
  "metadata": {
    "model": "command-r",
    "tokens": 198,
    "latency": 1320
  }
}
```

---

#### 4. Get Conversation History

```bash
curl http://localhost:3000/api/chat/conversations/550e8400-e29b-41d4-a716-446655440000
```

**Response**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "createdAt": "2026-02-10T12:00:00Z",
  "updatedAt": "2026-02-10T12:00:12Z",
  "messages": [
    {
      "id": "...",
      "role": "user",
      "content": "How do I implement error handling in Python?",
      "timestamp": "2026-02-10T12:00:00Z"
    },
    {
      "id": "...",
      "role": "assistant",
      "content": "Here's how to implement error handling...",
      "timestamp": "2026-02-10T12:00:05Z",
      "metadata": { "model": "command-r", "tokens": 245, "latency": 1450 }
    },
    {
      "id": "...",
      "role": "user",
      "content": "Can you show me an example with file operations?",
      "timestamp": "2026-02-10T12:00:10Z"
    },
    {
      "id": "...",
      "role": "assistant",
      "content": "Here's a complete example...",
      "timestamp": "2026-02-10T12:00:12Z",
      "metadata": { "model": "command-r", "tokens": 198, "latency": 1320 }
    }
  ],
  "context": {
    "conversationId": "550e8400-e29b-41d4-a716-446655440000",
    "summary": "",
    "topics": ["Python", "error handling", "file operations", "try-except"],
    "lastUpdated": "2026-02-10T12:00:12Z",
    "tokenCount": 1850
  },
  "status": "active"
}
```

---

#### 5. End Conversation

```bash
curl -X DELETE http://localhost:3000/api/chat/conversations/550e8400-e29b-41d4-a716-446655440000
```

**Response**: `204 No Content`

---

## Testing

### Automated Test Suite

```bash
# Run full test suite
npm test

# Run specific test file
npm test -- agent.test.ts

# Run tests in watch mode
npm test -- --watch

# Generate coverage report
npm run test:coverage
```

---

### Manual Conversation Testing

Use the provided test script:

```bash
npm run test:conversation
```

This script:
1. Creates a new conversation
2. Sends a series of test messages
3. Validates context maintenance
4. Tests boundary enforcement (medical/legal refusal)
5. Tests error handling scenarios
6. Generates a test report

**Sample Output**:
```
🧪 Running Conversation Tests

✅ Test 1: Technical Q&A
   Response time: 1.2s
   Context maintained: Yes

✅ Test 2: Step-by-step explanation
   Response format: Structured
   Code blocks present: Yes

✅ Test 3: Medical advice refusal
   Correctly refused: Yes
   Helpful redirection: Yes

✅ Test 4: Multi-turn context
   Context maintained: Yes
   Topics tracked: ["Python", "testing", "automation"]

✅ Test 5: Error handling
   Graceful degradation: Yes
   User-friendly message: Yes

📊 Results: 5/5 tests passed
```

---

## Troubleshooting

### Common Issues

#### 1. "Invalid API Key" Error

**Symptoms**:
```json
{
  "error": "INVALID_API_KEY",
  "message": "Service configuration error. Please contact support."
}
```

**Solutions**:
- Verify `COHERE_API_KEY` is set in `.env`
- Check API key is valid at [Cohere Dashboard](https://dashboard.cohere.com)
- Ensure no extra spaces or quotes around the key
- Try regenerating the API key

---

#### 2. "Rate Limit Exceeded" Error

**Symptoms**:
```json
{
  "error": "RATE_LIMIT_EXCEEDED",
  "message": "Service is busy. Please wait a moment and try again."
}
```

**Solutions**:
- Wait 60 seconds and retry (automatic exponential backoff implemented)
- Check your Cohere plan limits at [Cohere Dashboard](https://dashboard.cohere.com/billing)
- Consider upgrading to a higher tier plan
- Implement request queuing in your application

**Cohere Free Tier Limits**:
- 100 requests per minute
- 10,000 requests per month

---

#### 3. Slow Response Times

**Symptoms**:
- Responses taking >5 seconds
- Timeout errors

**Solutions**:
- Check network latency: `ping api.cohere.ai`
- Reduce `maxTokens` in agent config (shorter responses = faster)
- Use streaming responses for better perceived performance
- Monitor token usage (large contexts increase latency)
- Consider using `command-r` instead of `command-r-plus` (faster but slightly lower quality)

---

#### 4. Context Not Maintained

**Symptoms**:
- Assistant doesn't remember previous messages
- Responses don't reference earlier context

**Solutions**:
- Verify conversation ID is consistent across requests
- Check conversation hasn't expired (30-minute timeout)
- Ensure messages are being stored correctly (check logs)
- Verify context update logic is running

---

#### 5. Server Won't Start

**Symptoms**:
```
Error: Cannot find module 'cohere-ai'
```

**Solutions**:
```bash
# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install

# Verify Node.js version
node --version  # Should be 20.x or higher

# Check for port conflicts
lsof -i :3000  # Kill any process using port 3000
```

---

## Performance Optimization

### Response Time Optimization

```typescript
// Use streaming for long responses
const response = await cohere.chat({
  message: userInput,
  stream: true,  // Enable streaming
});

// Process chunks as they arrive
for await (const chunk of response) {
  process.stdout.write(chunk.text);
}
```

### Token Usage Optimization

```typescript
// Prune context aggressively
if (tokenCount > 50000) {
  pruneMessages(conversationId, 10);  // Keep only last 10 messages
}

// Use shorter system prompts
const CONCISE_SYSTEM_PROMPT = "You are a helpful AI assistant.";
```

### Caching Strategy

```typescript
// Cache system prompts (they don't change)
const systemPromptCache = new Map<string, string>();

// Cache common responses
const responseCache = new LRU({ max: 100 });
```

---

## Next Steps

### 1. Integrate Frontend UI

Navigate to `frontend/` directory and follow the setup instructions:

```bash
cd frontend
npm install
npm run dev
```

The frontend provides:
- Chat interface with message history
- Real-time typing indicators
- Code syntax highlighting
- Markdown rendering
- Mobile-responsive design

---

### 2. Implement Tool Calling

Add custom tools to extend agent capabilities:

```typescript
import { ITool } from './contracts/agent-interface';

const weatherTool: ITool = {
  name: 'get_weather',
  description: 'Get current weather for a location',
  parameters: {
    type: 'object',
    properties: {
      location: { type: 'string', description: 'City name' },
    },
    required: ['location'],
  },
  async execute(args) {
    const weather = await fetchWeather(args.location);
    return { success: true, data: weather };
  },
};

// Register tool with agent
agent.updateConfig({ tools: [weatherTool] });
```

---

### 3. Add Conversation Persistence

Integrate database for long-term storage:

```typescript
// PostgreSQL example
import { Pool } from 'pg';

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

async function saveConversation(conversation: Conversation) {
  await pool.query(
    'INSERT INTO conversations (id, messages, context, status) VALUES ($1, $2, $3, $4)',
    [conversation.id, JSON.stringify(conversation.messages), JSON.stringify(conversation.context), conversation.status]
  );
}
```

---

### 4. Deploy to Production

**Deployment Checklist**:
- [ ] Set production `COHERE_API_KEY`
- [ ] Configure production `NODE_ENV=production`
- [ ] Set up HTTPS/SSL certificates
- [ ] Configure CORS for frontend domain
- [ ] Set up monitoring (Datadog, New Relic, etc.)
- [ ] Configure logging (Winston, Pino)
- [ ] Set up error tracking (Sentry)
- [ ] Configure rate limiting
- [ ] Set up health check monitoring
- [ ] Configure auto-scaling

**Recommended Platforms**:
- **Vercel**: Easy deployment for Node.js APIs
- **Railway**: Simple deployment with automatic HTTPS
- **AWS ECS**: Scalable container deployment
- **Google Cloud Run**: Serverless container platform

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                     User Input                          │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              System Prompt Injection                    │
│  (Safety boundaries, response style, capabilities)      │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                Agent Reasoning                          │
│  (Agent.run → assembles context + prompt)               │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Cohere Completion                          │
│  (CohereAdapter.chat → API call)                        │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Tool Routing (if applicable)               │
│  (Runner routes to appropriate tool)                    │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              Final User Response                        │
│  (Formatted, context-aware, safe)                       │
└─────────────────────────────────────────────────────────┘
```

---

## Additional Resources

- **Cohere Documentation**: [https://docs.cohere.com](https://docs.cohere.com)
- **API Reference**: See `contracts/chat-api.yaml`
- **Architecture Details**: See `plan.md`
- **Data Model**: See `data-model.md`
- **Research Findings**: See `research.md`

---

## Support

For issues or questions:
1. Check this quickstart guide
2. Review troubleshooting section
3. Check API documentation (`contracts/chat-api.yaml`)
4. Review architecture plan (`plan.md`)
5. Open an issue on GitHub (if applicable)

---

**Version**: 1.0.0 | **Last Updated**: 2026-02-10
