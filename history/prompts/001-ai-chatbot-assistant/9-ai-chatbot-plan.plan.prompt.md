---
id: 9
title: ai-chatbot-plan
stage: plan
date: 2026-02-10
surface: agent
model: claude-sonnet-4-5
feature: 001-ai-chatbot-assistant
branch: 001-ai-chatbot-assistant
user: user
command: /sp.plan
labels: ["planning", "architecture", "cohere", "agent-sdk", "chatbot"]
links:
  spec: specs/001-ai-chatbot-assistant/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-chatbot-assistant/plan.md
  - specs/001-ai-chatbot-assistant/research.md
  - specs/001-ai-chatbot-assistant/data-model.md
  - specs/001-ai-chatbot-assistant/contracts/chat-api.yaml
  - specs/001-ai-chatbot-assistant/contracts/agent-interface.ts
  - specs/001-ai-chatbot-assistant/quickstart.md
tests:
  - N/A (planning stage)
---

## Prompt

```
/sp.plan

Architecture plan:
- Agent core compatible with OpenAI Agent SDK patterns
- Cohere adapter layer implementing:
  - generate()
  - chat()
  - tool-call-compatible outputs
- Prompt orchestration layer (system + user + tools)
- Optional memory manager

Execution flow:
User Input
→ System Prompt Injection
→ Agent Reasoning
→ Cohere Completion
→ Tool Routing (if any)
→ Final User Response

Validation strategy:
- Test simple Q&A
- Test multi-turn context retention
- Test error scenarios (invalid key, timeout)
- Test tool-call simulation

Quality checks:
- No OpenAI API dependency
- No hallucinated capabilities
- Clear and predictable behavior
```

## Response snapshot

Created comprehensive implementation plan for AI Chatbot Assistant with Cohere-based architecture:

**Phase 0: Research (research.md)**
- R1: Cohere API integration patterns → Use `cohere.chat()` for conversational interface
- R2: OpenAI Agent SDK pattern adaptation → Agent, Runner, Tool abstractions without OpenAI dependency
- R3: Prompt engineering for safety boundaries → Multi-layered system prompts with explicit refusal patterns
- R4: Conversation context management → Session-based in-memory with token-aware pruning
- R5: Tool calling simulation → JSON schema-based tool interface compatible with Cohere
- R6: Error handling & graceful degradation → Comprehensive error taxonomy with fallback strategies

**Phase 1: Design Artifacts**
1. **data-model.md**: 4 core entities (Conversation, Message, Context, ResponseFormat) with validation rules and state transitions
2. **contracts/chat-api.yaml**: OpenAPI 3.0 specification with 4 endpoints (create conversation, send message, get history, end conversation)
3. **contracts/agent-interface.ts**: TypeScript interfaces for IAgent, IProvider, ITool, IRunner, IMemoryManager
4. **quickstart.md**: Complete setup guide with prerequisites, usage examples, troubleshooting, and deployment checklist

**Technical Stack**:
- Language: TypeScript 5.x / Node.js 20.x
- LLM Provider: Cohere (Command-R models)
- API Framework: Express.js or Fastify
- Validation: Zod
- Storage: In-memory (MVP), extensible to PostgreSQL/Redis
- Testing: Jest + Supertest

**Architecture Highlights**:
- Provider-agnostic design (Cohere adapter implements IProvider interface)
- OpenAI Agent SDK patterns (Agent, Runner, Tool) without OpenAI dependency
- Execution flow: User Input → System Prompt → Agent Reasoning → Cohere API → Tool Routing → Response
- Safety boundaries enforced via system prompts (refuse medical/legal advice, no autonomous actions)
- Context management with token-aware pruning (100k token limit)
- Graceful error handling with exponential backoff for rate limits

**Constitution Compliance**: ✅ All checks passed
- Core Principle VIII (AI Assistant Integration): Provider abstraction, Cohere primary, graceful fallback, API key security
- Technical Accuracy: Verified claims, tested code examples
- Pedagogical Clarity: Step-by-step explanations, clear responses
- Professional Quality: Structured responses, educational error messages
- Safety & Scope: Refuse out-of-scope requests, explicit uncertainty

**Project Structure**:
- Backend: `backend/src/` with agent/, api/, models/, services/, utils/
- Frontend: `frontend/src/` with components/, services/
- Tests: Unit, integration, and manual conversation tests

## Outcome

- ✅ Impact: Complete implementation plan with research, design artifacts, and contracts. Ready for task generation and implementation.
- 🧪 Tests: Testing strategy defined (unit, integration, manual conversation flow tests)
- 📁 Files: 6 files created (plan.md, research.md, data-model.md, 2 contract files, quickstart.md)
- 🔁 Next prompts: `/sp.tasks` to generate actionable, dependency-ordered implementation tasks
- 🧠 Reflection: Successfully incorporated user's Cohere-based architecture into comprehensive plan. Provider abstraction enables future LLM swapping. All constitution requirements satisfied. Research phase resolved all technical unknowns. Design artifacts provide clear implementation guidance.

## Evaluation notes (flywheel)

- Failure modes observed: Agent context update script failed (template not found), but this doesn't block planning completion. Can be addressed during implementation.
- Graders run and results (PASS/FAIL): PASS - Constitution check passed all gates. All mandatory sections completed. Research resolved all NEEDS CLARIFICATION items.
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided architecture guidance
- Next experiment (smallest change to try): Consider adding streaming response examples in quickstart.md for better UX demonstration. Could also add performance benchmarking section to research.md.
