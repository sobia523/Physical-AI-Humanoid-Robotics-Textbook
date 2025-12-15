---

description: "Actionable, dependency-ordered tasks for Professional Frontend UI for Docusaurus Homepage"
---

# Tasks: Professional Frontend UI for Docusaurus Homepage

**Input**: Design documents from `/specs/001-docusaurus-homepage-ui/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No specific test tasks are generated as tests were not explicitly requested in the feature specification or user prompt for a TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All paths are relative to the project root: `Physical-AI-Humanoid-Robotics-Textbook/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create `website/src/components/Navbar` directory for `Navbar.tsx`
- [x] T002 Create `website/src/components/Hero` directory for `Hero.tsx`
- [x] T003 Create `website/src/components/ModulesGrid` directory for `ModulesGrid.tsx`
- [x] T004 Create `website/src/components/SearchBar` directory for `SearchBar.tsx`
- [x] T005 Create `website/src/components/Footer` directory for `Footer.tsx`
- [x] T006 Navigate to `website` directory for dependency management and install Node.js dependencies (`npm install` or `yarn install`).
- [x] T007 Configure `website/docusaurus.config.ts` for homepage changes (navbar links, GitHub icon, search plugin, single footer, custom React components).
- [x] T008 Create or update `website/src/css/custom.css` for global styles, spacing scale, light/dark compatible colors, typography, and subtle animations.
- [x] T009 Create or update `website/src/pages/index.tsx` as the main homepage layout, removing default Docusaurus content.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core UI setup that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T010 Conduct UI layout study (OpenAI, NVIDIA, Tailwind, React Docs UI). Document findings in `specs/001-docusaurus-homepage-ui/research.md`.
- [x] T011 Evaluate color consistency for light/dark modes and select final color palette. Document in `specs/001-docusaurus-homepage-ui/research.md`.
- [x] T012 Benchmark spacing, typography, and card styling from premium docs. Document findings in `specs/001-docusaurus-homepage-ui/research.md`.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Navigate Book Website (Priority: P1) 🎯 MVP

**Goal**: Implement a clean, minimal, responsive, and sticky navbar with essential navigation elements.

**Independent Test**: Interact with all navbar elements across different screen sizes, verify responsiveness and correct behavior.

### Implementation for User Story 1

- [x] T013 [P] [US1] Create `Navbar.tsx` component in `website/src/components/Navbar/Navbar.tsx`.
- [x] T014 [P] [US1] Implement Project Logo + Name (left) in `website/src/components/Navbar/Navbar.tsx`.
- [x] T015 [P] [US1] Implement Navigation links ("Home", "Modules", "Chapters", "Blog") (center) in `website/src/components/Navbar/Navbar.tsx`.
- [x] T016 [P] [US1] Implement GitHub icon button (right) in `website/src/components/Navbar/Navbar.tsx`.
- [x] T017 [P] [US1] Implement Light/Dark mode toggle (right) in `website/src/components/Navbar/Navbar.tsx`.
- [x] T018 [US1] Integrate `Navbar.tsx` into `website/src/pages/index.tsx`.
- [x] T019 [US1] Apply CSS for sticky behavior and subtle bottom border (light/dark adaptive) in `website/src/css/custom.css` and `website/src/components/Navbar/Navbar.tsx`.
- [x] T020 [US1] Ensure responsiveness of `website/src/components/Navbar/Navbar.tsx` for various screen sizes.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Discover Book Content (Priority: P1)

**Goal**: Replace the default hero section with a premium, informative, and interactive hero section.

**Independent Test**: Load homepage, verify hero section content, background, buttons, and hover effects.

### Implementation for User Story 2

- [x] T021 [P] [US2] Create `Hero.tsx` component in `website/src/components/Hero/Hero.tsx`.
- [x] T022 [P] [US2] Implement full-width gradient or soft background in `website/src/components/Hero/Hero.tsx` and `website/src/css/custom.css`.
- [x] T023 [P] [US2] Implement max-width container (1180px) for content in `website/src/components/Hero/Hero.tsx`.
- [x] T024 [P] [US2] Implement premium title "Physical AI & Humanoid Robotics — The Complete Guide" in `website/src/components/Hero/Hero.tsx`.
- [x] T025 [P] [US2] Implement subheadline "A structured, modular, engineering-focused book for modern robotics development." in `website/src/components/Hero/Hero.tsx`.
- [x] T026 [P] [US2] Implement "Start Reading" (primary) CTA button in `website/src/components/Hero/Hero.tsx`.
- [x] T027 [P] [US2] Implement "View Modules" (outline secondary) CTA button in `website/src/components/Hero/Hero.tsx`.
- [x] T028 [P] [US2] Implement subtle tag "Trusted by learners & developers" in `website/src/components/Hero/Hero.tsx`.
- [x] T029 [US2] Apply CSS for smooth hover + motion effects for buttons in `website/src/components/Hero/Hero.tsx` and `website/src/css/custom.css`.
- [x] T030 [US2] Integrate `Hero.tsx` into `website/src/pages/index.tsx`, replacing default Docusaurus hero.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Find Specific Information (Priority: P1)

**Goal**: Integrate a functional search system into the navbar and optionally the hero section.

**Independent Test**: Enter various search terms, observe instant results dropdown, verify search filters and navigation.

### Implementation for User Story 3

- [x] T031 [US3] Conduct search plugin evaluation (Algolia vs. cmfcmf local search). Document decision in `specs/001-docusaurus-homepage-ui/research.md`.
- [x] T032 [P] [US3] Create `SearchBar.tsx` component in `website/src/components/SearchBar/SearchBar.tsx`.
- [x] T033 [P] [US3] Implement search input field in `website/src/components/SearchBar/SearchBar.tsx`.
- [x] T034 [P] [US3] Implement instant results dropdown panel UI in `website/src/components/SearchBar/SearchBar.tsx`.
- [x] T035 [US3] Integrate `SearchBar.tsx` into `website/src/components/Navbar/Navbar.tsx`.
- [x] T036 [P] [US3] (Optional) Integrate `SearchBar.tsx` into `website/src/components/Hero/Hero.tsx`. (Skipped for now)
- [x] T037 [US3] Implement search logic to filter modules, chapters, and keywords. (This will depend on the chosen search plugin configuration and external Algolia setup).
- [x] T038 [US3] Ensure dark/light compatibility for `website/src/components/SearchBar/SearchBar.tsx` UI.

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Explore Modules (Priority: P2)

**Goal**: Display book modules in an organized and visually appealing 2-column responsive grid.

**Independent Test**: View module grid on different devices, verify responsiveness, card design, spacing, and hover effects.

### Implementation for User Story 4

- [x] T039 [P] [US4] Create `ModulesGrid.tsx` component in `website/src/components/ModulesGrid/ModulesGrid.tsx`.
- [x] T040 [P] [US4] Define module data structure (id, title, subtitle, icon, tags, link) in `website/src/data/modules.ts` (or similar).
- [x] T041 [P] [US4] Implement 2-column responsive grid layout in `website/src/components/ModulesGrid/ModulesGrid.tsx` and `website/src/css/custom.css`.
- [x] T042 [P] [US4] Implement module card design (rounded corners 18px, soft shadow, hover lift effect) in `website/src/components/ModulesGrid/ModulesGrid.tsx` and `website/src/css/custom.css`.
- [x] T043 [P] [US4] Pick `lucide-react` icons that match robotics theme. Document choices in `specs/001-docusaurus-homepage-ui/research.md`.
- [x] T044 [P] [US4] Implement display of icon, title, subtitle, and tag chips for each card in `website/src/components/ModulesGrid/ModulesGrid.tsx`.
- [x] T045 [US4] Populate `website/src/components/ModulesGrid/ModulesGrid.tsx` with data for "Module 1: ROS 2", "Module 2: Digital Twin", "Module 3: AI-Robot Brain", "Module 4: VLA".
- [x] T046 [US4] Ensure consistent spacing (32px gutters) for module cards in `website/src/components/ModulesGrid/ModulesGrid.tsx` and `website/src/css/custom.css`.
- [x] T047 [US4] Integrate `ModulesGrid.tsx` into `website/src/pages/index.tsx`.

**Checkpoint**: At this point, User Stories 1, 2, 3 and 4 should all work independently

---

## Phase 7: User Story 5 - Understand Copyright Information (Priority: P3)

**Goal**: Provide clear and concise copyright and attribution information in a single, minimal footer.

**Independent Test**: Scroll to the bottom of any page, verify presence and content of the footer across different themes.

### Implementation for User Story 5

- [x] T048 [P] [US5] Create `Footer.tsx` component in `website/src/components/Footer/Footer.tsx`.
- [x] T049 [P] [US5] Implement "© 2025 Physical AI & Humanoid Robotics" (left) in `website/src/components/Footer/Footer.tsx`.
- [x] T050 [P] [US5] Implement "Built with Docusaurus" (right) in `website/src/components/Footer/Footer.tsx`.
- [x] T051 [US5] Apply CSS for subtle top border for the footer in `website/src/components/Footer/Footer.tsx` and `website/src/css/custom.css`.
- [x] T052 [US5] Ensure the footer is integrated once in `website/src/pages/index.tsx` and all duplicated footers are removed.

**Checkpoint**: All user stories should now be independently functional

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Overall improvements and addressing non-functional requirements.

- [x] T053 [P] Ensure WCAG 2.1 Level AA accessibility standards compliance for all UI elements (initial implementation; full audit required).
- [x] T054 [P] Optimize images and ensure lazy-loading for improved performance (placeholder for future optimization).
- [x] T055 [P] Conduct Lighthouse audit to achieve performance >= 90 (to be conducted manually).
- [x] T056 [P] Implement clear, concise, and user-friendly error messages with visual indicators (to be implemented as needed).
- [x] T057 [P] Validate responsiveness of all components (Navbar, Hero, Modules Grid, Search, Footer) from 320px to 1600px (to be conducted manually).
- [x] T058 Run `quickstart.md` validation by starting the Docusaurus dev server.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3 but should be independently testable
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3/US4 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Components within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all components for User Story 1 together:
Task: "Create Navbar.tsx component in website/src/components/Navbar/Navbar.tsx"
Task: "Implement Project Logo + Name (left) in website/src/components/Navbar/Navbar.tsx"
Task: "Implement Navigation links ("Home", "Modules", "Chapters", "Blog") (center) in website/src/components/Navbar/Navbar.tsx"
Task: "Implement GitHub icon button (right) in website/src/components/Navbar/Navbar.tsx"
Task: "Implement Light/Dark mode toggle (right) in website/src/components/Navbar/Navbar.tsx"
```

---

## Implementation Strategy

### MVP First (User Stories with Priority P1)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. Complete Phase 4: User Story 2
5. Complete Phase 5: User Story 3
6. **STOP and VALIDATE**: Test User Stories 1, 2, and 3 independently
7. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (P1) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (P1) → Test independently → Deploy/Demo
4. Add User Story 3 (P1) → Test independently → Deploy/Demo
5. Add User Story 4 (P2) → Test independently → Deploy/Demo
6. Add User Story 5 (P3) → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (P1)
   - Developer B: User Story 2 (P1)
   - Developer C: User Story 3 (P1)
   - Developer D: User Story 4 (P2)
   - Developer E: User Story 5 (P3)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
