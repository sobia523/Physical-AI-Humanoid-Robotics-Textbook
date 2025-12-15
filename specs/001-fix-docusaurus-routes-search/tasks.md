---
description: "Actionable, dependency-ordered tasks for Fix Broken Routes & Enable Working Search in Docusaurus v3"
---

# Tasks: Fix Broken Routes & Enable Working Search in Docusaurus v3

**Input**: Design documents from `specs/001-fix-docusaurus-routes-search/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/, quickstart.md

**Tests**: No specific test tasks are generated as tests were not explicitly requested in the feature specification or user prompt for a TDD approach. Verification will be done through manual validation as per `quickstart.md`.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All paths are relative to the project root: `Physical-AI-Humanoid-Robotics-Textbook/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Ensure Docusaurus environment is ready for modifications.

- [X] T001 Ensure local Docusaurus development environment is set up and running in `website/`.
- [X] T002 Verify `website/package.json` for Docusaurus v3 compatibility and presence of `@docusaurus/Link`.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Audit current state before making changes.

- [X] T003 Audit existing internal links (Start Reading, module cards, navbar) within `website/` for broken routes. Document findings.
- [X] T004 Identify the current search implementation in `website/docusaurus.config.ts` and `website/src/components/` (if any). Document findings.
- [X] T005 Verify the existence of `/docs/tutorial-basics/create-a-document` or another guaranteed valid entry page in the Docusaurus site structure.

---

## Phase 3: User Story 1 - Navigate Book Content (Priority: P1) 🎯 Routing Fixes

**Goal**: Resolve "Page Not Found" errors by fixing internal links.

**Independent Test**: Verify all internal links (Start Reading, module cards, navbar) navigate to correct pages without 404 errors, as described in `quickstart.md`.

### Implementation for User Story 1

- [X] T006 [P] [US1] Update "Start Reading" button link to point to a guaranteed valid page (e.g., `/docs/tutorial-basics/create-a-document`) in `website/src/components/Hero/Hero.tsx` or similar component.
- [X] T007 [P] [US1] Audit and update module card links within `website/src/components/ModulesGrid/ModulesGrid.tsx` or associated data files to ensure they point to existing module pages.
- [X] T008 [P] [US1] Audit and update navbar links within `website/docusaurus.config.ts` or `website/src/components/Navbar/Navbar.tsx` to ensure they point to existing pages.
- [X] T009 [US1] Ensure all internal links use `@docusaurus/Link` component where appropriate in `website/src/`.
- [X] T010 [US1] Confirm no UI/CSS changes were introduced by routing fixes in `website/`.

---

## Phase 4: User Story 2 - Search Book Content (Priority: P1) 🎯 Search Fixes

**Goal**: Implement and configure functional global search.

**Independent Test**: Verify search bar returns relevant results for modules, chapters, and page headings, as described in `quickstart.md`.

### Implementation for User Story 2

- [X] T011 [US2] Decide on the official Docusaurus search solution: `@easyops-cn/docusaurus-search-local` or Algolia (requires credential check). Document choice in `research.md` if not already there.
- [X] T012 [US2] Install the chosen search package (e.g., `npm install @easyops-cn/docusaurus-search-local`) in `website/`.
- [X] T013 [US2] Register and configure the chosen search plugin in `website/docusaurus.config.ts`, ensuring it indexes documentation content.
- [X] T014 [US2] Remove any custom/fake search input components from `website/src/` to avoid conflicts.
- [X] T015 [US2] Ensure only the official Docusaurus search component is used.
- [X] T016 [US2] Confirm no custom JavaScript search logic was introduced in `website/`.
- [X] T017 [US2] Confirm no UI/CSS changes were introduced by search implementation in `website/`.

---

## Final Phase: Validation & Polish

**Purpose**: Verify all fixes and ensure stability.

- [X] T018 Run `npm install` (if necessary after search package install) and then `npm run start` in `website/` to restart the development server.
- [X] T019 Perform comprehensive manual verification of routing fixes as per `quickstart.md` (all links, no 404s).
- [X] T020 Perform comprehensive manual verification of search functionality as per `quickstart.md` (search terms, relevant results, no console errors).
- [X] T021 Build the Docusaurus site (`npm run build` in `website/`) and verify routing and search functionality in the production build (optional: serve `build/` directory).
- [X] T022 Ensure no console errors related to routing or search are present in both development and production builds.

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion.
-   **User Story 1 (Phase 3)**: Depends on Foundational completion.
-   **User Story 2 (Phase 4)**: Depends on Foundational completion. Can run in parallel with User Story 1 if different team members. If sequential, it depends on User Story 1.
-   **Final Phase (Phase 5)**: Depends on completion of User Story 1 and User Story 2.

### User Story Dependencies

-   **User Story 1 (P1)**: Independent of User Story 2, but both depend on Foundational tasks.
-   **User Story 2 (P1)**: Independent of User Story 1, but both depend on Foundational tasks.

### Parallel Opportunities

-   Tasks marked with `[P]` within a user story can be executed in parallel.
-   User Story 1 and User Story 2 can be worked on in parallel by different team members after the Foundational phase is complete.

---

## Implementation Strategy

### MVP First (User Stories with Priority P1)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 1 (Routing Fixes)
4.  Complete Phase 4: User Story 2 (Search Fixes)
5.  Complete Phase 5: Final Phase (Validation)

### Incremental Delivery

-   Each phase delivers verifiable improvements.
-   Validation (Phase 5) should be performed after both User Story phases are complete.

---

## Notes

-   Tasks `T003` and `T004` in the Foundational phase require documentation of findings.
-   Task `T011` in User Story 2 requires a decision on the search solution.
-   Verification will be manual as specified in `quickstart.md`.
