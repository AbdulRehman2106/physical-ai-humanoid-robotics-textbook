# AI Chatbot Integration

## Overview

The Physical AI Textbook includes an AI-powered chatbot assistant built with Cohere API and OpenAI Agent SDK-compatible architecture.

## Features

- ✅ **Provider Abstraction**: Easily swap LLM providers without changing agent logic
- ✅ **Cohere Integration**: Uses Cohere Command-R+ for high-quality responses
- ✅ **Graceful Error Handling**: User-friendly error messages and fallback mechanisms
- ✅ **Constitutional Compliance**: Follows Principle VIII (AI Assistant Integration)
- ✅ **Performance**: < 2s response time target
- ✅ **Security**: API keys via environment variables only
- ✅ **Textbook Context**: Specialized knowledge about Physical AI, ROS 2, simulation

## Setup

### 1. Get Cohere API Key

1. Visit [Cohere Dashboard](https://dashboard.cohere.com/api-keys)
2. Sign up or log in
3. Create a new API key (free tier available)

### 2. Configure Environment

```bash
# Copy the example environment file
cp .env.example .env.local

# Edit .env.local and add your API key
COHERE_API_KEY=your_actual_api_key_here
```

### 3. Build and Run

```bash
# Install dependencies (if not already done)
npm install

# Start development server
npm start

# Or build for production
npm run build
```

## Architecture

### Provider Abstraction Layer

```
src/lib/llm/
├── types.ts              # Interface definitions
├── cohereProvider.ts     # Cohere API implementation
├── agent.ts              # Agent and Runner classes
└── index.ts              # Exports
```

**Key Interfaces:**
- `LLMProvider`: Abstract provider interface
- `Agent`: Manages conversation and system prompts
- `Runner`: Executes agent with performance monitoring

### UI Component

```
src/components/Chatbot/
├── index.tsx             # React component
└── Chatbot.module.css    # Styling
```

**Features:**
- Floating chat button (bottom-right)
- Collapsible chat window
- Message history
- Typing indicator
- Error handling UI
- Mobile responsive

### Integration

```
src/theme/Root.tsx        # Global wrapper
```

The chatbot is integrated globally via Docusaurus theme swizzling.

## Usage

### For Users

1. Click the floating chat button (💬) in the bottom-right corner
2. Ask questions about:
   - Physical AI fundamentals
   - ROS 2 development
   - Robot simulation
   - VLA models
   - Sim-to-real transfer
   - Error handling
3. Get instant, context-aware answers

### For Developers

#### Creating a Custom Agent

```typescript
import { createCohereProvider, Agent } from '@site/src/lib/llm';

const provider = createCohereProvider();
const agent = new Agent({
  name: 'Custom Agent',
  instructions: 'Your system prompt here',
  provider,
  temperature: 0.7,
  maxTokens: 1000,
});

const response = await agent.run('User message');
console.log(response.content);
```

#### Switching Providers

To add a new provider (e.g., OpenAI):

1. Create `src/lib/llm/openaiProvider.ts`
2. Implement the `LLMProvider` interface
3. Update agent initialization to use new provider

```typescript
import { OpenAIProvider } from './openaiProvider';

const provider = new OpenAIProvider(apiKey);
const agent = createTextbookAgent(provider);
```

## Configuration

### Agent Configuration

Edit `src/lib/llm/agent.ts` to customize:

```typescript
export function createTextbookAgent(provider: LLMProvider): Agent {
  return new Agent({
    name: 'Physical AI Textbook Assistant',
    instructions: `Your custom system prompt...`,
    provider,
    temperature: 0.7,      // Adjust creativity (0-1)
    maxTokens: 1000,       // Max response length
  });
}
```

### UI Customization

Edit `src/components/Chatbot/Chatbot.module.css` to change:
- Colors and gradients
- Button position
- Chat window size
- Animation effects

## Performance

### Monitoring

The `Runner` class logs performance warnings:

```typescript
// Logs warning if response takes > 2s
const runner = new Runner(agent);
const response = await runner.execute(userMessage);
```

### Optimization Tips

1. **Reduce maxTokens**: Shorter responses = faster
2. **Adjust temperature**: Lower = more deterministic = faster
3. **Use caching**: Implement response caching for common questions
4. **Batch requests**: For multiple questions, consider batching

## Error Handling

### Graceful Degradation

The chatbot handles errors gracefully:

1. **API Key Missing**: Shows configuration message
2. **Network Error**: Shows retry message
3. **Rate Limit**: Shows wait message
4. **Timeout**: Shows timeout message

### Error Messages

All errors are user-friendly:

```typescript
// Instead of: "ProviderError: 401 Unauthorized"
// Shows: "I'm sorry, but the AI assistant is not configured..."
```

## Security

### API Key Protection

✅ **DO:**
- Store API key in `.env.local`
- Use environment variables
- Add `.env*` to `.gitignore`

❌ **DON'T:**
- Commit API keys to git
- Hardcode keys in source code
- Share keys publicly

### Rate Limiting

Cohere free tier limits:
- 100 requests/minute
- 10,000 requests/month

Consider implementing:
- Client-side rate limiting
- Request queuing
- Usage analytics

## Testing

### Manual Testing

1. Start dev server: `npm start`
2. Open chatbot
3. Test scenarios:
   - Ask about ROS 2
   - Ask about VLA models
   - Test error handling (invalid API key)
   - Test mobile responsiveness

### Automated Testing

```bash
# Add tests in tests/chatbot.test.ts
npm test
```

## Troubleshooting

### Chatbot Not Appearing

1. Check `src/theme/Root.tsx` exists
2. Verify Docusaurus theme swizzling
3. Clear cache: `npm run clear`

### API Errors

1. Verify API key in `.env.local`
2. Check Cohere dashboard for quota
3. Test API key with curl:

```bash
curl -X POST https://api.cohere.ai/v1/chat \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "model": "command-r-plus"}'
```

### Build Errors

1. Check TypeScript errors: `npm run typecheck`
2. Verify all imports are correct
3. Clear cache and rebuild: `npm run clear && npm run build`

## Constitutional Compliance

This implementation follows **Constitutional Principle VIII: AI Assistant Integration**:

- ✅ Provider abstraction layer
- ✅ Cohere as primary provider
- ✅ OpenAI Agent SDK-compatible patterns
- ✅ Graceful error handling
- ✅ No hallucinated facts (system prompt enforces)
- ✅ Environment variable API keys
- ✅ < 2s response time target
- ✅ Textbook-specific context

## Future Enhancements

### Planned Features

1. **Conversation History**: Persist across sessions
2. **Code Execution**: Run code examples in chat
3. **Chapter Context**: Auto-inject current chapter context
4. **Voice Input**: Speech-to-text integration
5. **Multi-language**: Support for multiple languages
6. **Analytics**: Track common questions
7. **Feedback**: Thumbs up/down on responses

### Provider Expansion

- OpenAI GPT-4
- Anthropic Claude
- Google Gemini
- Local models (Ollama)

## Support

For issues or questions:
- Check this documentation
- Review constitution: `.specify/memory/constitution.md`
- Check Cohere docs: https://docs.cohere.com/
- Open GitHub issue

## License

Same as the main textbook project.

---

**Version**: 1.0.0
**Last Updated**: 2026-02-10
**Constitutional Version**: 1.1.0
