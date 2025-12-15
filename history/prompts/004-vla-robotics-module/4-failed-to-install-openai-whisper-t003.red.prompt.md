---
id: 4
title: Failed to install openai-whisper (T003)
stage: red
date: 2025-12-15
surface: agent
model: gemini-cli
feature: 004-vla-robotics-module
branch: 004-vla-robotics-module
user: TLS
command: /sp.implement
labels: [vla-robotics, installation, failure]
links:
  spec: ../../../../specs/004-vla-robotics-module/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-vla-robotics-module/tasks.md
tests:
  - none
---

## Prompt

```text
$ARGUMENTS
```

## Response snapshot

The installation of `openai-whisper` (task T003) failed due to an `OSError: [Errno 2] No such file or directory` during the installation of its dependency `torch`. This typically occurs on Windows systems when the installation path exceeds the maximum path length limit.

I am unable to programmatically enable Windows long path support or shorten the project's base directory path. Therefore, I am unable to complete task T003 and any subsequent tasks that depend on `openai-whisper`.

## Outcome

- ❌ Impact: Unable to proceed with implementation due to a critical dependency installation failure.
- 🧪 Tests: No functional tests could be run.
- 📁 Files:
  - `specs/004-vla-robotics-module/tasks.md` (T003 remains incomplete)
- 🔁 Next prompts: User needs to resolve the underlying Windows long path issue before restarting the `/sp.implement` command.
- 🧠 Reflection: Identified a critical environment-specific issue (Windows long path limitation) that prevented progress. The agent's capabilities are limited in resolving such system-level configurations.

## Evaluation notes (flywheel)

- Failure modes observed: `OSError: [Errno 2] No such file or directory` during `pip install openai-whisper` (due to `torch` dependency) likely caused by Windows path length limit.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): User resolving system configuration issue.
