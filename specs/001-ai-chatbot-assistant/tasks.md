# Tasks: AI Chatbot Assistant

**Input**: Design documents from `/specs/001-ai-chatbot-assistant/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/, research.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**Tests**: Not explicitly requested in specification - test tasks omitted per guidelines.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `frontend/src/`, `frontend/tests/`
- Paths follow web application structure from plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend project structure with src/ and tests/ directories per plan.md
- [ ] T002 Initialize Node.js project with package.json in backend/ directory
- [ ] T003 [P] Install core dependencies: cohere-ai, express, zod, uuid, dotenv in backend/package.json
- [ ] T004 [P] Install dev dependencies: typescript, jest, supertest, @types/* in backend/package.json
- [ ] T005 [P] Configure TypeScript with tsconfig.json in backend/ directory
- [ ] T006 [P] Configure Jest with jest.config.js in backend/ directory
- [ ] T007 [P] Create .env.example file in backend/ with COHERE_API_KEY, PORT, NODE_ENV placeholders
- [ ] T008 [P] Add .gitignore with .env, node_modules, dist/ entries in backend/ directory
- [ ] T009 Create frontend project structure with src/ directory per plan.md
- [ ] T010 Initialize React/Next.js project with package.json in frontend/ directory

**Checkpoint**: Project structure ready for foundational implementation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Configuration & Utilities

- [ ] T011 [P] Implement environment configuration loader in backend/src/utils/config.ts
- [ ] T012 [P] Implement logger utility with Winston or Pino in backend/src/utils/logger.ts
- [ ] T013 [P] Create error classes (AgentError, ProviderError, ToolError) in backend/src/utils/errors.ts

### Agent Core Interfaces

- [ ] T014 [P] Define IAgent interface in backend/src/agent/core/Agent.ts per contracts/agent-interface.ts
- [ ] T015 [P] Define IProvider interface in backend/src/agent/providers/ProviderInterface.ts per contracts/agent-interface.ts
- [ ] T016 [P] Define ITool interface in backend/src/agent/core/Tool.ts per contracts/agent-interface.ts
- [ ] T017 [P] Define IRunner interface in backend/src/agent/core/Runner.ts per contracts/agent-interface.ts
- [ ] T018 [P] Define IMemoryManager interface in backend/src/agent/memory/MemoryManager.ts per contracts/agent-interface.ts
- [ ] T019 [P] Define IContext interface and supporting types in backend/src/agent/core/types.ts

### Cohere Provider Implementation

- [ ] T020 Implement CohereAdapter class in backend/src/agent/providers/CohereAdapter.ts implementing IProvider
- [ ] T021 Add Cohere SDK initialization with API key from environment in CohereAdapter constructor
- [ ] T022 Implement chat() method in CohereAdapter using cohere.chat() per research.md findings
- [ ] T023 Implement complete() method in CohereAdapter using cohere.generate() per research.md
- [ ] T024 Implement healthCheck() method in CohereAdapter to verify API connectivity
- [ ] T025 Add error handling with exponential backoff for rate limits in CohereAdapter per research.md
- [ ] T026 Add timeout handling (10s) for Cohere API calls in CohereAdapter

### System Prompts & Safety

- [ ] T027 Create system prompt template with safety boundaries in backend/src/agent/prompts/SystemPrompts.ts per research.md
- [ ] T028 Add refusal patterns for medical/legal/financial advice in SystemPrompts.ts
- [ ] T029 Add uncertainty admission patterns in SystemPrompts.ts
- [ ] T030 Add autonomous action refusal patterns in SystemPrompts.ts
- [ ] T031 Implement PromptOrchestrator class in backend/src/agent/prompts/PromptOrchestrator.ts
- [ ] T032 Add assembleMessages() method in PromptOrchestrator to combine system + user + history

### API Server Infrastructure

- [ ] T033 Create Express server setup in backend/src/api/server.ts with CORS and JSON middleware
- [ ] T034 [P] Implement error handling middleware in backend/src/api/middleware/errorHandler.ts
- [ ] T035 [P] Implement request validation middleware using Zod in backend/src/api/middleware/validation.ts
- [ ] T036 [P] Create health check endpoint GET /api/health in backend/src/api/routes/health.ts
- [ ] T037 Add Cohere provider health check to /api/health endpoint

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Technical Question Answering (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask technical questions and receive accurate, context-aware responses with code examples

**Independent Test**: Submit technical questions (Python error handling, JavaScript async/await, database design) and verify responses are accurate, include code examples, and maintain context across follow-ups

### Data Models

- [ ] T038 [P] [US1] Create Conversation interface in backend/src/models/Conversation.ts per data-model.md
- [ ] T039 [P] [US1] Create Message interface in backend/src/models/Message.ts per data-model.md
- [ ] T040 [P] [US1] Create Context interface in backend/src/models/Context.ts per data-model.md
- [ ] T041 [P] [US1] Add validation functions for Conversation in backend/src/models/Conversation.ts
- [ ] T042 [P] [US1] Add validation functions for Message in backend/src/models/Message.ts

### Memory Management

- [ ] T043 [US1] Implement ConversationMemory class in backend/src/agent/memory/ConversationMemory.ts
- [ ] T044 [US1] Add in-memory Map storage for conversations in ConversationMemory
- [ ] T045 [US1] Implement addMessage() method in ConversationMemory per data-model.md
- [ ] T046 [US1] Implement getMessages() method in ConversationMemory
- [ ] T047 [US1] Implement getContext() method in ConversationMemory
- [ ] T048 [US1] Implement updateContext() method with topic extraction in ConversationMemory
- [ ] T049 [US1] Add session timeout logic (30 minutes) in ConversationMemory
- [ ] T050 [US1] Add cleanup job for expired sessions in ConversationMemory

### Agent Implementation

- [ ] T051 [US1] Implement ChatAgent class in backend/src/agent/core/ChatAgent.ts implementing IAgent
- [ ] T052 [US1] Implement run() method in ChatAgent that calls provider.chat() per plan.md execution flow
- [ ] T053 [US1] Add response format detection in ChatAgent.run() per data-model.md
- [ ] T054 [US1] Add metadata tracking (tokens, latency, model) in ChatAgent.run()
- [ ] T055 [US1] Implement getConfig() and updateConfig() methods in ChatAgent

### Runner Implementation

- [ ] T056 [US1] Implement Runner class in backend/src/agent/core/ChatRunner.ts implementing IRunner
- [ ] T057 [US1] Implement createConversation() method in Runner
- [ ] T058 [US1] Implement run() method in Runner that orchestrates Agent + Memory per plan.md
- [ ] T059 [US1] Add context loading and updating in Runner.run()
- [ ] T060 [US1] Implement endConversation() method in Runner
- [ ] T061 [US1] Implement getContext() method in Runner

### Business Logic

- [ ] T062 [US1] Implement ChatService class in backend/src/services/ChatService.ts
- [ ] T063 [US1] Add createConversation() method in ChatService that calls Runner
- [ ] T064 [US1] Add sendMessage() method in ChatService that validates input and calls Runner
- [ ] T065 [US1] Add getConversation() method in ChatService
- [ ] T066 [US1] Add endConversation() method in ChatService
- [ ] T067 [US1] Add input validation (max 10k chars, non-empty) in ChatService.sendMessage()

### API Endpoints

- [ ] T068 [US1] Implement POST /api/chat/conversations endpoint in backend/src/api/routes/chat.ts per contracts/chat-api.yaml
- [ ] T069 [US1] Implement POST /api/chat/conversations/:id/messages endpoint in backend/src/api/routes/chat.ts
- [ ] T070 [US1] Implement GET /api/chat/conversations/:id endpoint in backend/src/api/routes/chat.ts
- [ ] T071 [US1] Implement DELETE /api/chat/conversations/:id endpoint in backend/src/api/routes/chat.ts
- [ ] T072 [US1] Add request validation for all chat endpoints using validation middleware
- [ ] T073 [US1] Add error handling for all chat endpoints (404, 400, 500) per contracts/chat-api.yaml
- [ ] T074 [US1] Register chat routes in backend/src/api/server.ts

### Integration

- [ ] T075 [US1] Wire up ChatService with Runner, Agent, Provider, and Memory in backend/src/api/server.ts
- [ ] T076 [US1] Add environment variable validation for COHERE_API_KEY on startup
- [ ] T077 [US1] Test end-to-end flow: create conversation → send message → verify response

**Checkpoint**: User Story 1 complete - Core Q&A functionality working independently

---

## Phase 4: User Story 2 - Step-by-Step Guidance (Priority: P1)

**Goal**: Enable users to request step-by-step explanations that break down complex topics into logical, sequential steps

**Independent Test**: Request step-by-step explanations (REST APIs, sorting algorithms, Docker setup) and verify responses use numbered steps with clear progression from basic to advanced

### Prompt Engineering

- [ ] T078 [P] [US2] Add step-by-step instruction template to SystemPrompts.ts
- [ ] T079 [P] [US2] Add few-shot examples for step-by-step responses in SystemPrompts.ts
- [ ] T080 [US2] Add step expansion patterns for follow-up clarifications in SystemPrompts.ts

### Response Enhancement

- [ ] T081 [US2] Add detectStepByStepRequest() method in PromptOrchestrator to identify step-by-step queries
- [ ] T082 [US2] Enhance system prompt injection in PromptOrchestrator when step-by-step detected
- [ ] T083 [US2] Add step numbering validation in response format detection

### Integration

- [ ] T084 [US2] Update ChatAgent.run() to use enhanced prompts for step-by-step requests
- [ ] T085 [US2] Test step-by-step flow: request explanation → verify numbered steps → request clarification → verify expansion

**Checkpoint**: User Story 2 complete - Step-by-step guidance working independently

---

## Phase 5: User Story 5 - Safe and Responsible Guidance (Priority: P1)

**Goal**: Ensure chatbot refuses out-of-scope requests (medical, legal, autonomous actions) and admits uncertainty appropriately

**Independent Test**: Submit prohibited requests (medical diagnosis, legal advice, autonomous actions) and verify chatbot refuses with helpful redirection; submit uncertain queries and verify explicit uncertainty admission

### Safety Service

- [ ] T086 [P] [US5] Implement SafetyService class in backend/src/services/SafetyService.ts
- [ ] T087 [P] [US5] Add detectProhibitedTopic() method in SafetyService (medical, legal, financial, autonomous)
- [ ] T088 [P] [US5] Add generateRefusalMessage() method in SafetyService with helpful redirection
- [ ] T089 [P] [US5] Add detectUncertainty() method in SafetyService based on response content

### Integration

- [ ] T090 [US5] Integrate SafetyService into ChatService.sendMessage() for pre-processing
- [ ] T091 [US5] Add post-processing check in ChatService for uncertainty patterns
- [ ] T092 [US5] Add logging for all safety refusals and uncertainty admissions
- [ ] T093 [US5] Test safety boundaries: medical → refuse, legal → refuse, autonomous → refuse, uncertain → admit

**Checkpoint**: User Story 5 complete - Safety boundaries enforced independently

---

## Phase 6: User Story 3 - Conversational Q&A with Context (Priority: P2)

**Goal**: Enable multi-turn conversations where chatbot maintains context and provides coherent, contextually-aware responses

**Independent Test**: Conduct multi-turn conversation (5+ exchanges) where each question references previous answers, verify context maintained and responses consistent

### Context Management Enhancement

- [ ] T094 [P] [US3] Add estimateTokens() method in ConversationMemory per research.md
- [ ] T095 [P] [US3] Implement pruneMessages() method in ConversationMemory with token-aware pruning
- [ ] T096 [P] [US3] Add summarizeMessages() method in ConversationMemory using Cohere
- [ ] T097 [US3] Add automatic pruning trigger when tokenCount > 100k in ConversationMemory.addMessage()
- [ ] T098 [US3] Add context summary injection as system message when pruning occurs

### Context Tracking

- [ ] T099 [US3] Enhance updateContext() in ConversationMemory to extract topics from recent messages
- [ ] T100 [US3] Add context shift detection in ConversationMemory.updateContext()
- [ ] T101 [US3] Add context metadata to Message objects (topics, summary reference)

### Integration

- [ ] T102 [US3] Update Runner.run() to handle pruned context with summaries
- [ ] T103 [US3] Test multi-turn context: 10 exchanges → verify context maintained → exceed token limit → verify pruning → verify summary used

**Checkpoint**: User Story 3 complete - Context maintenance working independently

---

## Phase 7: User Story 4 - Professional and Structured Responses (Priority: P2)

**Goal**: Ensure responses use appropriate formatting (headers, lists, code blocks, tables) for improved readability

**Independent Test**: Ask questions requiring structured answers (comparisons, pros/cons, code examples) and verify responses use appropriate formatting

### Response Format Detection

- [ ] T104 [P] [US4] Create ResponseFormat type and FormattedResponse interface in backend/src/models/ResponseFormat.ts per data-model.md
- [ ] T105 [P] [US4] Implement detectFormat() function in backend/src/models/ResponseFormat.ts
- [ ] T106 [P] [US4] Add format-specific metadata detection (hasCodeBlocks, hasLists, hasHeaders)

### Prompt Enhancement

- [ ] T107 [US4] Add formatting guidelines to SystemPrompts.ts (when to use headers, lists, code blocks, tables)
- [ ] T108 [US4] Add few-shot examples for each format type in SystemPrompts.ts

### Integration

- [ ] T109 [US4] Update ChatAgent.run() to detect and tag response format
- [ ] T110 [US4] Add format metadata to Message.metadata in ChatService
- [ ] T111 [US4] Test structured responses: comparison → verify table, code question → verify code blocks, pros/cons → verify lists

**Checkpoint**: User Story 4 complete - Structured responses working independently

---

## Phase 8: Frontend UI (Optional - Can be separate project)

**Purpose**: Provide user interface for chatbot interaction

- [ ] T112 [P] Create ChatInterface component in frontend/src/components/ChatInterface.tsx
- [ ] T113 [P] Create MessageList component in frontend/src/components/MessageList.tsx
- [ ] T114 [P] Create InputBox component in frontend/src/components/InputBox.tsx
- [ ] T115 [P] Implement chatApi service in frontend/src/services/chatApi.ts
- [ ] T116 Integrate components in frontend/src/App.tsx
- [ ] T117 [P] Add markdown rendering for assistant messages
- [ ] T118 [P] Add syntax highlighting for code blocks
- [ ] T119 [P] Add loading indicators and error handling
- [ ] T120 Test frontend: create conversation → send messages → verify display → verify formatting

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T121 [P] Add comprehensive logging for all agent operations in ChatAgent and Runner
- [ ] T122 [P] Add performance monitoring (latency tracking) in ChatService
- [ ] T123 [P] Add rate limiting middleware in backend/src/api/middleware/rateLimit.ts
- [ ] T124 [P] Create API documentation from contracts/chat-api.yaml using Swagger UI
- [ ] T125 [P] Add request/response examples to API documentation
- [ ] T126 [P] Create conversation test script per quickstart.md in backend/scripts/test-conversation.ts
- [ ] T127 Run quickstart.md validation: setup → create conversation → send messages → verify all scenarios
- [ ] T128 [P] Add security headers (helmet.js) to Express server
- [ ] T129 [P] Add input sanitization for XSS prevention
- [ ] T130 Code cleanup: remove console.logs, add JSDoc comments, format with Prettier

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-7)**: All depend on Foundational phase completion
  - US1 (Phase 3): Can start after Foundational - No dependencies on other stories
  - US2 (Phase 4): Can start after Foundational - Enhances US1 but independently testable
  - US5 (Phase 5): Can start after Foundational - Integrates with US1 but independently testable
  - US3 (Phase 6): Can start after Foundational - Enhances US1 but independently testable
  - US4 (Phase 7): Can start after Foundational - Enhances US1 but independently testable
- **Frontend (Phase 8)**: Depends on US1 completion (needs working API)
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Foundation only - Core MVP
- **User Story 2 (P1)**: Foundation only - Prompt enhancement
- **User Story 5 (P1)**: Foundation only - Safety layer
- **User Story 3 (P2)**: Foundation only - Context enhancement
- **User Story 4 (P2)**: Foundation only - Format enhancement

**All user stories are independently testable and can be developed in parallel after Foundational phase**

### Within Each User Story

- Models before services
- Services before API endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task T038: "Create Conversation interface in backend/src/models/Conversation.ts"
Task T039: "Create Message interface in backend/src/models/Message.ts"
Task T040: "Create Context interface in backend/src/models/Context.ts"
Task T041: "Add validation functions for Conversation"
Task T042: "Add validation functions for Message"

# Launch all API endpoints for User Story 1 together (after services complete):
Task T068: "Implement POST /api/chat/conversations endpoint"
Task T069: "Implement POST /api/chat/conversations/:id/messages endpoint"
Task T070: "Implement GET /api/chat/conversations/:id endpoint"
Task T071: "Implement DELETE /api/chat/conversations/:id endpoint"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 5 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Technical Q&A)
4. Complete Phase 4: User Story 2 (Step-by-Step)
5. Complete Phase 5: User Story 5 (Safety)
6. **STOP and VALIDATE**: Test all three stories independently
7. Deploy/demo MVP

**MVP Scope**: Core Q&A with step-by-step explanations and safety boundaries

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (Core MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (Enhanced MVP)
4. Add User Story 5 → Test independently → Deploy/Demo (Safe MVP)
5. Add User Story 3 → Test independently → Deploy/Demo (Context-aware)
6. Add User Story 4 → Test independently → Deploy/Demo (Professional formatting)
7. Add Frontend → Test independently → Deploy/Demo (Full product)
8. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Core Q&A)
   - Developer B: User Story 2 (Step-by-Step)
   - Developer C: User Story 5 (Safety)
3. After P1 stories complete:
   - Developer A: User Story 3 (Context)
   - Developer B: User Story 4 (Formatting)
   - Developer C: Frontend UI
4. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests not included per specification (not explicitly requested)
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Total tasks: 130 tasks across 9 phases
- MVP tasks (Phases 1-5): 93 tasks
- Full implementation: 130 tasks

---

## Task Count Summary

- **Phase 1 (Setup)**: 10 tasks
- **Phase 2 (Foundational)**: 27 tasks
- **Phase 3 (US1 - Technical Q&A)**: 40 tasks
- **Phase 4 (US2 - Step-by-Step)**: 8 tasks
- **Phase 5 (US5 - Safety)**: 8 tasks
- **Phase 6 (US3 - Context)**: 10 tasks
- **Phase 7 (US4 - Formatting)**: 8 tasks
- **Phase 8 (Frontend)**: 9 tasks
- **Phase 9 (Polish)**: 10 tasks

**Total**: 130 tasks

**MVP (US1 + US2 + US5)**: 93 tasks
**Full Backend**: 111 tasks
**Full Product**: 130 tasks
