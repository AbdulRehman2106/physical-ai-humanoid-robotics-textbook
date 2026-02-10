---
id: 7
title: Add AI chatbot integration principle
stage: constitution
date: 2026-02-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-physical-ai-book
user: ar204
command: /sp.constitution
labels: ["constitution", "ai-chatbot", "cohere", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Constitution version updated to 1.1.0
  - New principle VIII added
  - AI Chatbot technical standards added
  - Quality gates updated
---

## Prompt

User requested to create an AI chatbot using OpenAI Agent SDK architecture but with Cohere API instead of OpenAI API. User invoked `/sp.constitution` to establish governance principles for the chatbot integration.

**User Requirements:**
- AI Chatbot built using OpenAI Agent SDK architecture with Cohere LLM backend
- Provider abstraction: LLM provider must be interchangeable
- Reliability: Stable responses with graceful fallback
- Clarity: Responses optimized for human readability
- Safety: No hallucinated facts; admit uncertainty explicitly
- Performance: Low-latency conversational flow
- Cohere API key ONLY (no OpenAI key dependency)
- Agent pattern: OpenAI Agent SDK-compatible (Agent, Runner, Tool abstraction)
- Stateless by default; memory optional and explicit
- Environment-variable based API key handling

## Response

**Constitution Amendment Summary:**

**Version Change:** 1.0.0 → 1.1.0 (MINOR bump)
- **Rationale:** New principle added (Principle VIII: AI Assistant Integration) without breaking existing principles

**Changes Made:**

1. **Added Core Principle VIII: AI Assistant Integration**
   - Provider-agnostic architecture requirement
   - Cohere as primary provider with OpenAI SDK patterns
   - Graceful error handling and fallback mechanisms
   - No hallucinated facts; explicit uncertainty
   - Stateless conversational memory by default
   - Environment variable API key handling
   - Response quality standards

2. **Added Technical Standards: AI Chatbot Requirements**
   - Provider: Cohere (Command / Command-R / Command-R+)
   - Architecture: OpenAI Agent SDK-compatible patterns
   - API Key: Environment variable `COHERE_API_KEY`
   - Performance: < 2s response time target
   - Safety: No hallucinations, explicit uncertainty
   - Context: Textbook-specific knowledge

3. **Updated Quality Gates**
   - Added AI Chatbot Compliance checklist
   - Cohere API integration testing
   - Provider abstraction verification
   - Error handling validation
   - Response accuracy checks
   - API key security audit
   - Performance testing

4. **Updated Sync Impact Report**
   - Documented version change
   - Listed added sections
   - Flagged templates needing updates
   - Added follow-up TODOs

**Files Modified:**
- `.specify/memory/constitution.md` - Constitution updated to v1.1.0

**Templates Requiring Updates:**
- ⚠️ Plan template - needs alignment with AI assistant requirements
- ⚠️ Spec template - needs alignment with chatbot integration
- ⚠️ Tasks template - needs AI assistant implementation tasks

**Follow-up TODOs:**
- Set up Cohere API key environment variable
- Implement provider abstraction layer
- Create chatbot UI component
- Test graceful fallback mechanisms
- Document chatbot usage in user guide

## Outcome

- ✅ Constitution successfully amended to v1.1.0
- ✅ New principle VIII added for AI Assistant Integration
- ✅ Technical standards established for Cohere-based chatbot
- ✅ Quality gates updated with AI chatbot compliance checks
- ✅ Sync Impact Report documented
- 📋 Templates flagged for alignment updates
- 🔄 Ready to proceed with chatbot implementation following constitutional principles

## Evaluation

**Success Criteria Met:**
- ✅ Provider abstraction principle established
- ✅ Cohere API as primary provider documented
- ✅ OpenAI Agent SDK compatibility pattern defined
- ✅ Error handling and safety requirements specified
- ✅ Performance targets set (< 2s response time)
- ✅ API key security requirements documented
- ✅ Quality gates include chatbot testing

**Next Steps:**
1. Implement provider abstraction layer
2. Create Cohere API integration
3. Build chatbot UI component
4. Test error handling and fallbacks
5. Validate response accuracy
6. Deploy with environment variable configuration

**Governance Impact:**
- MINOR version bump appropriate (new principle, no breaking changes)
- All existing principles remain intact
- New principle complements existing quality and accessibility standards
- Constitution remains internally consistent
