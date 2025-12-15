---
id: 2
title: Generate Docusaurus Routes Search Plan
stage: plan
date: 2025-12-13
surface: agent
model: gemini-pro
feature: 001-fix-docusaurus-routes-search
branch: 001-fix-docusaurus-routes-search
user: unknown
command: /sp.plan Fix Broken Routes & Enable Working Search (Docusaurus v3)
labels: ["plan", "docusaurus", "routing", "search"]
links:
  spec: specs/001-fix-docusaurus-routes-search/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-fix-docusaurus-routes-search/plan.md
  - specs/001-fix-docusaurus-routes-search/research.md
  - specs/001-fix-docusaurus-routes-search/data-model.md
  - specs/001-fix-docusaurus-routes-search/contracts/README.md
  - specs/001-fix-docusaurus-routes-search/quickstart.md
  - GEMINI.md
tests: null
---

## Prompt

/sp.plan Fix Broken Routes & Enable Working Search (Docusaurus v3)

Objective:
- Fix “Page Not Found” errors caused by invalid internal links.
- Make the existing search bar fully functional for modules and chapters.
- Keep current UI, layout, navbar, footer, and styling unchanged.

--------------------------------------------------
Phase 1 — Routing Fix (Page Not Found)
--------------------------------------------------

1. Audit all internal links:
   - Start Reading button
   - Module cards links
   - Navbar links

2. Verify existing valid routes:
   - Check `/docs` structure
   - Identify at least ONE valid entry page (e.g. `/docs/intro`)

3. Fix links:
   - Use `@docusaurus/Link` everywhere
   - Point Start Reading to a guaranteed existing page:
     `/docs/intro`
   - Remove or update links pointing to non-existent modules

4. Validation:
   - Click Start Reading → no 404
   - Click navbar links → no 404

--------------------------------------------------
Phase 2 — Search Functionality Fix
--------------------------------------------------

1. Identify active search solution:
   - Preferred: `@easyops-cn/docusaurus-search-local`
   - Or Algolia (if credentials exist)

2. Configuration check:
   - Ensure search plugin is registered in `docusaurus.config.js`
   - Ensure docs are included in indexing

3. Remove conflicts:
   - Remove any custom/fake search input components
   - Use ONLY official Docusaurus search component

4. Rebuild search index:
   - Stop dev server
   - Clear cache:
     `.docusaurus`, `build`, `node_modules/.cache`
   - Restart with `npm start`

--------------------------------------------------
Phase 3 — Validation & Stability
--------------------------------------------------

1. Search test:
   - Search module titles
   - Search chapter titles
   - Search page headings

2. Confirm:
   - Search results appear
   - No console errors
   - No UI duplication
   - No Page Not Found screens

--------------------------------------------------
Outcome:
- Internal navigation fully working.
- Search bar properly indexing and returning results.
- UI untouched and stable.
- Ready for adding modules later without breaking navigation.

## Response snapshot

The implementation plan for "Fix Broken Routes & Enable Working Search in Docusaurus v3" has been successfully generated.

## Outcome

- ✅ Impact: A detailed implementation plan, research, data model, and quickstart guide have been generated for the feature, and the agent's context has been updated.
- 🧪 Tests: No automated tests were run as part of the planning phase.
- 📁 Files: Generated plan.md, research.md, data-model.md, contracts/README.md, quickstart.md, and updated GEMINI.md.
- 🔁 Next prompts: Next: /sp.tasks
- 🧠 Reflection: Successfully executed the planning workflow, generating all required design artifacts and updating the agent's context based on the feature's technical context.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Consider if any architectural decisions were made that warrant an ADR.
