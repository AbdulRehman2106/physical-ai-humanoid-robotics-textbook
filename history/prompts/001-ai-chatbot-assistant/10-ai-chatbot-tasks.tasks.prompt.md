---
id: 10
title: ai-chatbot-tasks
stage: tasks
date: 2026-02-10
surface: agent
model: claude-sonnet-4-5
feature: 001-ai-chatbot-assistant
branch: 001-ai-chatbot-assistant
user: user
command: /sp.tasks
labels: ["tasks", "implementation", "user-stories", "cohere", "agent-sdk"]
links:
  spec: specs/001-ai-chatbot-assistant/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-chatbot-assistant/tasks.md
tests:
  - N/A (task generation stage)
---

## Prompt

```
/sp.tasks

Build an AI chatbot that:
- Uses Cohere as the language model provider
- Follows OpenAI Agent SDK mental model
- Is extensible with tools and memory
- Produces professional-grade conversational output
```

## Response snapshot

Generated comprehensive task list for AI Chatbot Assistant with 130 actionable tasks organized by user story:

**Task Organization**:
- **Phase 1 - Setup**: 10 tasks (project initialization, dependencies, configuration)
- **Phase 2 - Foundational**: 27 tasks (core infrastructure that blocks all user stories)
- **Phase 3 - User Story 1** (Technical Q&A, P1): 40 tasks - MVP core functionality
- **Phase 4 - User Story 2** (Step-by-Step, P1): 8 tasks - prompt engineering for structured explanations
- **Phase 5 - User Story 5** (Safety, P1): 8 tasks - safety boundaries and refusal patterns
- **Phase 6 - User Story 3** (Context, P2): 10 tasks - multi-turn context management
- **Phase 7 - User Story 4** (Formatting, P2): 8 tasks - structured response formatting
- **Phase 8 - Frontend UI**: 9 tasks (optional, separate from backend MVP)
- **Phase 9 - Polish**: 10 tasks (cross-cutting concerns)

**Key Architectural Tasks**:
1. **Foundational Infrastructure** (T011-T037):
   - Configuration & utilities (config.ts, logger.ts, errors.ts)
   - Agent core interfaces (IAgent, IProvider, ITool, IRunner, IMemoryManager)
   - Cohere provider implementation (CohereAdapter with chat(), complete(), healthCheck())
   - System prompts with safety boundaries (medical/legal/autonomous refusal patterns)
   - API server infrastructure (Express, error handling, validation, health check)

2. **User Story 1 - Technical Q&A** (T038-T077):
   - Data models (Conversation, Message, Context with validation)
   - Memory management (ConversationMemory with session storage, timeout, cleanup)
   - Agent implementation (ChatAgent.run() with format detection, metadata tracking)
   - Runner implementation (orchestrates Agent + Memory + Provider)
   - Business logic (ChatService with input validation)
   - API endpoints (POST /conversations, POST /messages, GET /conversation, DELETE /conversation)
   - End-to-end integration and testing

3. **User Story 2 - Step-by-Step** (T078-T085):
   - Step-by-step prompt templates and few-shot examples
   - Request detection and prompt enhancement
   - Step numbering validation

4. **User Story 5 - Safety** (T086-T093):
   - SafetyService with topic detection (medical, legal, financial, autonomous)
   - Refusal message generation with helpful redirection
   - Uncertainty detection and admission
   - Pre/post-processing integration

5. **User Story 3 - Context** (T094-T103):
   - Token estimation and pruning (100k token limit)
   - Message summarization using Cohere
   - Topic extraction and context shift detection
   - Automatic pruning with summary injection

6. **User Story 4 - Formatting** (T104-T111):
   - Response format detection (plain, structured, code, table)
   - Format-specific metadata (hasCodeBlocks, hasLists, hasHeaders)
   - Formatting guidelines in system prompts

**Format Compliance**:
- All 130 tasks follow strict checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`
- 45 tasks marked [P] for parallel execution
- All tasks include exact file paths (backend/src/..., frontend/src/...)
- User story labels ([US1], [US2], etc.) for traceability

**MVP Scope**: 93 tasks (Phases 1-5)
- Delivers working chatbot with core Q&A, step-by-step explanations, and safety enforcement
- Independently testable at each user story checkpoint
- Can be deployed after US1 completion for early feedback

**Parallel Opportunities**:
- Setup phase: 6 parallel tasks (dependencies, config files)
- Foundational phase: 15 parallel tasks (interfaces, utilities, middleware)
- User Story 1: 8 parallel tasks (models, API endpoints)
- All user stories can run in parallel after Foundational phase completes

**Independent Testing Criteria**:
- US1: Submit technical questions, verify accurate responses with code examples, test context maintenance
- US2: Request step-by-step explanations, verify numbered steps with logical progression
- US5: Submit prohibited requests, verify refusals; submit uncertain queries, verify uncertainty admission
- US3: Conduct 5+ turn conversation, verify context maintained across exchanges
- US4: Ask questions requiring structure, verify appropriate formatting (headers, lists, code blocks, tables)

## Outcome

- ✅ Impact: Complete, actionable task list ready for implementation. 130 tasks organized by user story enable independent development and testing. MVP clearly defined (93 tasks).
- 🧪 Tests: Test tasks not included per specification (not explicitly requested). Each user story has independent test criteria defined.
- 📁 Files: 1 file created (tasks.md with 130 tasks)
- 🔁 Next prompts: `/sp.implement` to begin automated implementation, or manually execute tasks starting with Phase 1 (Setup)
- 🧠 Reflection: Successfully organized tasks by user story for independent implementation. All tasks follow strict checklist format with IDs, parallel markers, story labels, and file paths. Foundational phase clearly identified as blocking prerequisite. MVP scope (93 tasks) provides clear stopping point for early deployment. Parallel opportunities identified (45 tasks) for team efficiency.

## Evaluation notes (flywheel)

- Failure modes observed: None. Task generation completed successfully with proper organization and format compliance.
- Graders run and results (PASS/FAIL): PASS - All 130 tasks follow checklist format. User stories independently testable. MVP scope clearly defined. Parallel opportunities identified. File paths included for all tasks.
- Prompt variant (if applicable): Standard /sp.tasks workflow with user-provided architecture context
- Next experiment (smallest change to try): Consider adding estimated time ranges for each phase (e.g., "Phase 1: 2-4 hours") to help with sprint planning. Could also add skill level indicators (junior/mid/senior) for task assignment.
