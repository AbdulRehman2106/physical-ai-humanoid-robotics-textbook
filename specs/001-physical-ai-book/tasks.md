# Tasks: Physical AI & Humanoid Robotics Digital Book

**Input**: Design documents from `/specs/001-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/, research.md, quickstart.md

**Tests**: Tests are OPTIONAL for educational content projects. This project focuses on content quality validation rather than automated testing.

**Organization**: Tasks are grouped by user story (learning module) to enable independent implementation and testing of each module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Content**: `docs/chapters/[##-chapter-name]/`
- **Components**: `src/components/`
- **Styles**: `src/css/`
- **Static assets**: `static/img/`, `static/code-examples/`
- **Skills**: `.claude/skills/` (already exists)

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize Docusaurus project and basic structure

- [x] T001 Initialize Docusaurus project with TypeScript and MDX support in project root
- [x] T002 Configure docusaurus.config.js with book metadata, navigation, and theme settings
- [x] T003 [P] Create custom CSS theme in src/css/custom.css following ui-ux/book-layout-design skill
- [x] T004 [P] Set up directory structure: docs/chapters/, src/components/, static/img/, static/code-examples/
- [x] T005 [P] Configure Git ignore for node_modules, build artifacts, and temporary files
- [x] T006 Install dependencies: React 18+, Framer Motion, syntax highlighting, accessibility tools

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY content creation can begin

**⚠️ CRITICAL**: No content work can begin until this phase is complete

- [x] T007 Create base MDX component library structure in src/components/
- [x] T008 [P] Implement Callout component (info, tip, warning, danger, insight variants) in src/components/Callout/
- [x] T009 [P] Implement CodePlayground component with syntax highlighting in src/components/CodePlayground/
- [x] T010 [P] Implement Quiz component with immediate feedback in src/components/Quiz/
- [x] T011 [P] Implement InteractiveDiagram component in src/components/InteractiveDiagram/
- [x] T012 [P] Implement Checkpoint component for self-assessment in src/components/Checkpoint/
- [x] T013 [P] Configure scroll-based animation system using motion/scroll-animation skill
- [x] T014 [P] Set up accessibility testing pipeline (axe, WAVE, Lighthouse) with npm scripts
- [x] T015 [P] Create chapter template structure in docs/chapters/_template/
- [x] T016 [P] Set up code example testing framework with pytest in tests/code-examples/
- [x] T017 Configure Flesch-Kincaid readability scoring tool
- [x] T018 Create citation management system and bibliography template

**Checkpoint**: Foundation ready - content creation can now begin in parallel

---

## Phase 3: User Story 1 - Learn Physical AI Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Create foundational chapters (1-2) covering Physical AI principles and embodied intelligence

**Independent Test**: Students can complete Chapter 1-2 assessments and explain Physical AI concepts, embodied intelligence, and sensor-motor loops

### Implementation for User Story 1

- [x] T019 [P] [US1] Create Chapter 1 specification using spec/chapter-specification skill in docs/chapters/01-physical-ai-intro/
- [x] T020 [P] [US1] Define learning outcomes for Chapter 1 using spec/learning-outcomes skill
- [x] T021 [US1] Write Chapter 1 content using content/technical-chapter-writing and robotics/physical-ai-concepts skills in docs/chapters/01-physical-ai-intro/index.mdx
- [x] T022 [P] [US1] Create Physical AI landscape diagram using content/visual-content-description skill in static/img/diagrams/physical-ai-landscape.svg
- [x] T023 [P] [US1] Create embodied intelligence illustration in static/img/diagrams/embodied-intelligence.svg
- [x] T024 [P] [US1] Design concept animation for sensor-motor loop using motion/concept-animation skill
- [x] T025 [US1] Create Chapter 1 assessment using content/exercise-assessment-design skill in docs/chapters/01-physical-ai-intro/assessment.mdx
- [x] T026 [US1] Write Chapter 1 summary using content/summary-synthesis skill
- [x] T027 [P] [US1] Create Chapter 2 specification for embodied intelligence in docs/chapters/02-embodied-intelligence/
- [x] T028 [US1] Write Chapter 2 content on embodied intelligence and reality gap in docs/chapters/02-embodied-intelligence/index.mdx
- [x] T029 [P] [US1] Create reality gap comparison diagram in static/img/diagrams/reality-gap.svg
- [x] T030 [US1] Create Chapter 2 assessment and summary
- [x] T031 [US1] Validate US1 chapters against quality checklist (Flesch-Kincaid, accessibility, citations)
- [x] T032 [US1] Add APA citations for all Physical AI claims in Chapters 1-2

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Students can learn Physical AI fundamentals.

---

## Phase 4: User Story 2 - Master ROS 2 Development (Priority: P1)

**Goal**: Create ROS 2 tutorial chapters (3-4) covering nodes, communication patterns, lifecycle, and QoS

**Independent Test**: Students can create ROS 2 nodes, implement pub-sub communication, and pass code-based assessments

### Implementation for User Story 2

- [x] T033 [P] [US2] Create Chapter 3 specification for ROS 2 fundamentals in docs/chapters/03-ros2-fundamentals/
- [x] T034 [P] [US2] Define learning outcomes for ROS 2 chapters using spec/learning-outcomes skill
- [x] T035 [US2] Write Chapter 3 content using robotics/ros2-explanation and content/technical-chapter-writing skills in docs/chapters/03-ros2-fundamentals/index.mdx
- [ ] T036 [P] [US2] Create ROS 2 architecture diagram in static/img/diagrams/ros2-architecture.svg
- [ ] T037 [P] [US2] Create ROS 2 graph visualization diagram in static/img/diagrams/ros2-graph.svg
- [x] T038 [P] [US2] Design Hello World node code example using content/code-example-design skill in static/code-examples/ros2/hello_node.py
- [x] T039 [P] [US2] Create simple publisher code example in static/code-examples/ros2/simple_publisher.py
- [x] T040 [P] [US2] Create simple subscriber code example in static/code-examples/ros2/simple_subscriber.py
- [ ] T041 [US2] Test all Chapter 3 code examples in ROS 2 Humble environment
- [ ] T042 [US2] Create Chapter 3 hands-on tutorial using content/hands-on-tutorial-design skill
- [ ] T043 [US2] Create Chapter 3 assessment with code exercises
- [ ] T044 [P] [US2] Create Chapter 4 specification for communication patterns in docs/chapters/04-ros2-communication/
- [ ] T045 [US2] Write Chapter 4 content on topics, services, and actions in docs/chapters/04-ros2-communication/index.mdx
- [ ] T046 [P] [US2] Create communication patterns comparison diagram in static/img/diagrams/communication-patterns.svg
- [ ] T047 [P] [US2] Design animated data flow visualization using motion/concept-animation skill
- [ ] T048 [P] [US2] Create topic publisher/subscriber code examples in static/code-examples/ros2/
- [ ] T049 [P] [US2] Create service client/server code examples in static/code-examples/ros2/
- [ ] T050 [P] [US2] Create action client/server code examples in static/code-examples/ros2/
- [ ] T051 [US2] Test all Chapter 4 code examples
- [ ] T052 [US2] Create Chapter 4 assessment and exercises
- [ ] T053 [P] [US2] Create QoS configuration examples in static/code-examples/ros2/qos_examples.py
- [ ] T054 [P] [US2] Create lifecycle node implementation example in static/code-examples/ros2/lifecycle_node.py
- [ ] T055 [US2] Test QoS and lifecycle code examples
- [ ] T060 [US2] Create Chapter 4 section on QoS policies and lifecycle management
- [ ] T061 [US2] Create ROS 2 troubleshooting guide using error-handling/troubleshooting-guides skill in docs/chapters/04-ros2-communication/troubleshooting.mdx
- [ ] T062 [US2] Validate US2 chapters against quality checklist
- [ ] T063 [US2] Add APA citations for ROS 2 documentation references

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Students can learn Physical AI fundamentals and master ROS 2 development.

---

## Phase 5: User Story 3 - Implement Simulation Workflows (Priority: P2)

**Goal**: Create simulation chapters (5-7) covering Gazebo, Isaac Sim, and Unity

**Independent Test**: Students can set up simulation environments, create robot models, and run simulations

### Implementation for User Story 3

- [ ] T060 [P] [US3] Create Chapter 5 specification for simulation introduction in docs/chapters/05-simulation-intro/
- [ ] T061 [US3] Write Chapter 5 content using robotics/simulation-platforms skill in docs/chapters/05-simulation-intro/index.mdx
- [ ] T062 [P] [US3] Create simulation platform comparison matrix diagram in static/img/diagrams/simulation-comparison.svg
- [ ] T063 [US3] Create Chapter 5 assessment comparing simulation platforms
- [ ] T064 [P] [US3] Create Chapter 6 specification for Gazebo in docs/chapters/06-gazebo-basics/
- [ ] T065 [US3] Write Chapter 6 content on Gazebo simulation in docs/chapters/06-gazebo-basics/index.mdx
- [ ] T066 [P] [US3] Create Gazebo architecture diagram in static/img/diagrams/gazebo-architecture.svg
- [ ] T067 [P] [US3] Create URDF robot model example in static/code-examples/gazebo/robot_model.urdf
- [ ] T068 [P] [US3] Create Gazebo sensor plugin example in static/code-examples/gazebo/sensor_plugin.py
- [ ] T069 [P] [US3] Create Gazebo launch file example in static/code-examples/gazebo/robot.launch.py
- [ ] T070 [US3] Test all Gazebo examples in Gazebo Classic 11
- [ ] T071 [US3] Create Gazebo hands-on tutorial using content/hands-on-tutorial-design skill
- [ ] T072 [US3] Create Chapter 6 simulation exercise (build mobile robot)
- [ ] T073 [US3] Create Gazebo troubleshooting guide using error-handling/educational-error-explanation skill
- [ ] T074 [P] [US3] Create Chapter 7 specification for Isaac Sim in docs/chapters/07-isaac-sim/
- [ ] T075 [US3] Write Chapter 7 content on NVIDIA Isaac Sim in docs/chapters/07-isaac-sim/index.mdx
- [ ] T076 [P] [US3] Create Isaac Sim architecture diagram in static/img/diagrams/isaac-sim-architecture.svg
- [ ] T077 [P] [US3] Create Isaac Sim Python API example in static/code-examples/isaac-sim/basic_scene.py
- [ ] T078 [P] [US3] Create Isaac Sim ROS 2 bridge example in static/code-examples/isaac-sim/ros2_bridge.py
- [ ] T079 [US3] Test Isaac Sim examples (cloud environment if no local GPU)
- [ ] T080 [US3] Create Chapter 7 simulation exercise (robot arm with vision)
- [ ] T081 [US3] Validate US3 chapters against quality checklist
- [ ] T082 [US3] Add citations for simulation platform documentation

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently. Students can learn fundamentals, ROS 2, and simulation.

---

## Phase 6: User Story 4 - Apply Vision-Language-Action Models (Priority: P2)

**Goal**: Create VLA chapter (8) covering multimodal AI for robotics

**Independent Test**: Students can understand VLA architecture and implement basic VLA pipelines

### Implementation for User Story 4

- [ ] T083 [P] [US4] Create Chapter 8 specification for VLA models in docs/chapters/08-vla-models/
- [ ] T084 [US4] Write Chapter 8 content using robotics/physical-ai-concepts skill in docs/chapters/08-vla-models/index.mdx
- [ ] T085 [P] [US4] Create VLA pipeline architecture diagram in static/img/diagrams/vla-pipeline.svg
- [ ] T086 [P] [US4] Create VLA model architecture diagram in static/img/diagrams/vla-architecture.svg
- [ ] T087 [P] [US4] Design VLA inference animation using motion/concept-animation skill
- [ ] T088 [P] [US4] Create VLA inference code example in static/code-examples/vla/inference.py
- [ ] T089 [P] [US4] Create language-conditioned control example in static/code-examples/vla/language_control.py
- [ ] T090 [US4] Test VLA code examples (may require model weights or mock implementation)
- [ ] T091 [US4] Create Chapter 8 assessment on VLA concepts
- [ ] T092 [US4] Validate US4 chapter against quality checklist
- [ ] T093 [US4] Add citations for VLA research papers

**Checkpoint**: User Stories 1-4 complete. Students can learn fundamentals, ROS 2, simulation, and VLA models.

---

## Phase 7: User Story 5 - Execute Sim-to-Real Transfer (Priority: P3)

**Goal**: Create sim-to-real chapters (9-10) covering transfer techniques and Digital Twins

**Independent Test**: Students can understand sim-to-real challenges and implement domain randomization

### Implementation for User Story 5

- [ ] T094 [P] [US5] Create Chapter 9 specification for sim-to-real transfer in docs/chapters/09-sim-to-real/
- [ ] T095 [US5] Write Chapter 9 content using robotics/physical-ai-concepts skill in docs/chapters/09-sim-to-real/index.mdx
- [ ] T096 [P] [US5] Create sim-to-real pipeline diagram in static/img/diagrams/sim-to-real-pipeline.svg
- [ ] T097 [P] [US5] Create domain randomization visualization in static/img/diagrams/domain-randomization.svg
- [ ] T098 [P] [US5] Create domain randomization code example in static/code-examples/sim-to-real/domain_randomization.py
- [ ] T099 [P] [US5] Create system identification example in static/code-examples/sim-to-real/system_id.py
- [ ] T100 [US5] Test sim-to-real code examples
- [ ] T101 [US5] Create Chapter 9 assessment on transfer techniques
- [ ] T102 [P] [US5] Create Chapter 10 specification for Digital Twins in docs/chapters/10-digital-twins/
- [ ] T103 [US5] Write Chapter 10 content on Digital Twins in docs/chapters/10-digital-twins/index.mdx
- [ ] T104 [P] [US5] Create Digital Twin architecture diagram in static/img/diagrams/digital-twin-architecture.svg
- [ ] T105 [P] [US5] Create Digital Twin synchronization code example in static/code-examples/digital-twin/sync.py
- [ ] T106 [US5] Test Digital Twin examples
- [ ] T107 [US5] Create Chapter 10 simulation exercise (create Digital Twin)
- [ ] T108 [US5] Validate US5 chapters against quality checklist
- [ ] T109 [US5] Add citations for sim-to-real research papers

**Checkpoint**: All user stories complete. Students have full learning path from fundamentals to advanced topics.

---

## Phase 8: Error Handling & Troubleshooting (Supporting Content)

**Goal**: Create comprehensive error handling and troubleshooting chapter (11)

**Independent Test**: Students can diagnose and resolve common ROS 2, Gazebo, and Isaac Sim issues

- [ ] T110 [P] Create Chapter 11 specification for error handling in docs/chapters/11-error-handling/
- [ ] T111 Write Chapter 11 content using error-handling/educational-error-explanation skill in docs/chapters/11-error-handling/index.mdx
- [ ] T112 [P] Create troubleshooting decision tree diagram in static/img/diagrams/troubleshooting-tree.svg
- [ ] T113 [P] Create ROS 2 error catalog with solutions in docs/chapters/11-error-handling/ros2-errors.mdx
- [ ] T114 [P] Create Gazebo error catalog with solutions in docs/chapters/11-error-handling/gazebo-errors.mdx
- [ ] T115 [P] Create Isaac Sim error catalog with solutions in docs/chapters/11-error-handling/isaac-errors.mdx
- [ ] T116 Verify all error examples are accurate and solutions work
- [ ] T117 Create Chapter 11 assessment (debugging scenarios)
- [ ] T118 Validate Chapter 11 against quality checklist

**Checkpoint**: Error handling support complete. Students have comprehensive troubleshooting resources.

---

## Phase 9: Capstone Project (Integrative Assessment)

**Goal**: Create capstone project (Chapter 12) integrating all learning outcomes

**Independent Test**: Students can complete multi-phase capstone project demonstrating mastery

- [ ] T119 [P] Create Chapter 12 specification for capstone project in docs/chapters/12-capstone-project/
- [ ] T120 Design capstone project using capstone/project-design skill in docs/chapters/12-capstone-project/index.mdx
- [ ] T121 [P] Create capstone project architecture diagram in static/img/diagrams/capstone-architecture.svg
- [ ] T122 [P] Create project phase breakdown and timeline
- [ ] T123 [P] Create starter code template for capstone in static/code-examples/capstone/starter/
- [ ] T124 [P] Create capstone assessment rubric using capstone/assessment-rubrics skill in docs/chapters/12-capstone-project/rubric.md
- [ ] T125 Create capstone project milestones and checkpoints
- [ ] T126 Write capstone project guidance and hints
- [ ] T127 [P] Create example capstone solution (reference implementation) in static/code-examples/capstone/solution/
- [ ] T128 Test capstone starter code and solution
- [ ] T129 Validate capstone project against quality checklist

**Checkpoint**: Capstone project complete. Students have integrative final project.

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple chapters and final quality assurance

- [ ] T130 [P] Create homepage with course overview and navigation in docs/index.mdx
- [ ] T131 [P] Create glossary of technical terms in docs/glossary.mdx
- [ ] T132 [P] Create resources page with links to documentation in docs/resources.mdx
- [ ] T133 [P] Implement navigation enhancements using ui-ux/navigation-design skill
- [ ] T134 [P] Add progress tracking component in src/components/ProgressTracker/
- [ ] T135 [P] Create PDF export configuration for offline reading
- [ ] T136 Validate all internal links across chapters
- [ ] T137 Validate all external links (ROS 2 docs, papers, etc.)
- [ ] T138 Run Flesch-Kincaid readability check on all chapters (target: 10-12)
- [ ] T139 Run accessibility audit on all pages (axe, WAVE, Lighthouse)
- [ ] T140 Test keyboard navigation across entire site
- [ ] T141 Test with screen readers (NVDA, VoiceOver)
- [ ] T142 Cross-browser testing (Chrome, Firefox, Safari, Edge)
- [ ] T143 Mobile responsiveness testing (phone, tablet)
- [ ] T144 Performance optimization (Lighthouse score 90+)
- [ ] T145 [P] Create instructor guide for educators in docs/instructor-guide.mdx
- [ ] T146 [P] Create FAQ page in docs/faq.mdx
- [ ] T147 Final constitution compliance review across all content
- [ ] T148 Create comprehensive bibliography with all citations
- [ ] T149 Generate PDF export of complete book

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all content creation
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if multiple content creators)
  - Or sequentially in priority order (P1 → P1 → P2 → P2 → P3)
- **Error Handling (Phase 8)**: Can proceed after US2 (ROS 2) complete
- **Capstone (Phase 9)**: Depends on all user stories (US1-US5) being complete
- **Polish (Phase 10)**: Depends on all content being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent of US1 but logically follows
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Builds on US2 conceptually but independently testable
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Builds on US3 conceptually but independently testable
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - Synthesizes US1-US4 but independently testable

### Within Each User Story

- Chapter specifications before content writing
- Content writing before code examples (to understand context)
- Code examples before testing
- Diagrams can be created in parallel with content
- Assessments after content complete
- Validation after all chapter components complete

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Within each story, tasks marked [P] can run in parallel
- Different user stories can be worked on in parallel by different content creators

---

## Parallel Example: User Story 1

```bash
# Launch all diagram creation for User Story 1 together:
Task: "Create Physical AI landscape diagram in static/img/diagrams/physical-ai-landscape.svg"
Task: "Create embodied intelligence illustration in static/img/diagrams/embodied-intelligence.svg"

# Launch chapter specifications together:
Task: "Create Chapter 1 specification in docs/chapters/01-physical-ai-intro/"
Task: "Create Chapter 2 specification in docs/chapters/02-embodied-intelligence/"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all content)
3. Complete Phase 3: User Story 1 (Physical AI Fundamentals)
4. **STOP and VALIDATE**: Test Chapters 1-2 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Add Error Handling → Enhance support
8. Add Capstone → Complete course
9. Polish → Final release

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Creator A: User Story 1 (Physical AI Fundamentals)
   - Creator B: User Story 2 (ROS 2 Development)
   - Creator C: User Story 3 (Simulation Workflows)
3. Stories complete and integrate independently
4. Creators D & E: User Stories 4 & 5
5. Team: Error Handling, Capstone, Polish

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Skills library (`.claude/skills/`) provides guidance for all content creation tasks
- Follow constitution principles for all work
- Use contract templates for consistency
- Validate against quality checklists regularly

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 12 tasks
- **Phase 3 (US1 - Physical AI Fundamentals)**: 14 tasks
- **Phase 4 (US2 - ROS 2 Development)**: 23 tasks
- **Phase 5 (US3 - Simulation Workflows)**: 23 tasks
- **Phase 6 (US4 - VLA Models)**: 11 tasks
- **Phase 7 (US5 - Sim-to-Real Transfer)**: 16 tasks
- **Phase 8 (Error Handling)**: 9 tasks
- **Phase 9 (Capstone Project)**: 11 tasks
- **Phase 10 (Polish)**: 20 tasks

**Total**: 145 tasks

**Parallel Opportunities**: 58 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phases 1-3 (32 tasks) deliver foundational Physical AI content
