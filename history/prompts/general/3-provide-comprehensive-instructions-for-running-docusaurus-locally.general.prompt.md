---
id: 3
title: Provide comprehensive instructions for running Docusaurus locally
stage: general
date: 2025-12-17
surface: agent
model: claude-3-opus
feature: none
branch: main
user: 
command: |
  run on local host
labels:
  - user-guidance
  - troubleshooting
  - docusaurus
files:
  - website/.env
  - website/src/components/ChatWidget.js
tests: []
prompt_text: |
  run on local host
response_text: |
  The goal is to get the Docusaurus website running and accessible on your local machine.

  Here are comprehensive instructions to help you achieve this and troubleshoot the issues you've encountered:

  **Step-by-Step Guide to Running Your Docusaurus Website:**

  1.  **Ensure the `.env` file is correct:**
      *   Navigate to your project's `website` directory (`Physical-AI-Humanoid-Robotics-Textbook\website`).
      *   Open the `.env` file (if it exists) or create it if it doesn't.
      *   Ensure the file contains exactly this line:
          ```
          REACT_APP_RAG_AGENT_API_URL=http://localhost:8000/agent/ask
          ```
      *   Save the file. This is crucial for resolving the "process is not defined" error.

  2.  **Clear your browser's cache:**
      *   Before opening your browser, completely clear your browser's cache and cookies for `localhost` or `localhost:3001`. This ensures that your browser loads the latest version of the website code, which now correctly includes the environment variable.

  3.  **Start the Docusaurus Development Server (in your own terminal):**
      *   Open your operating system's terminal (e.g., PowerShell, Command Prompt, Git Bash).
      *   Navigate to the `website` directory of your project using the `cd` command:
          ```bash
          cd C:\Users\TLS\Downloads\geminiclispeckitplus\Physical-AI-Humanoid-Robotics-Textbook\website
          ```
      *   Run the development server command:
          ```bash
          npm run start
          ```
      *   **Observe the terminal output carefully.** You should see messages indicating compilation progress. Eventually, you should see a line similar to this:
          ```
          [SUCCESS] Docusaurus website is running at: http://localhost:3001/
          ```
          If you see any errors during this step, please copy and paste the *entire* error message here.

  4.  **Access the Website:**
      *   Once the terminal shows the `[SUCCESS]` message, open your web browser.
      *   Go directly to: `http://localhost:3001/`

  **If you still encounter issues:**

  *   **"process is not defined" error:** This should be resolved by Step 1 and Step 2. If it persists, double-check your `.env` file content and confirm your browser cache was cleared.
  *   **`ERR_CONNECTION_REFUSED` error:**
      *   Ensure the `npm run start` command is still actively running in your terminal window. Do not close that terminal window.
      *   Verify you are trying to access `http://localhost:3001/` and not `http://localhost:3000/` or any other port.
      *   Briefly check if any local firewall or proxy settings might be blocking access to `localhost:3001`.

  Please follow these steps carefully and let me know the outcome, especially if you encounter any error messages in your terminal.
outcome: |
  The user requested to run the Docusaurus development server locally. Due to previous issues with `ERR_CONNECTION_REFUSED` and "process is not defined", a comprehensive set of instructions was provided. These instructions guide the user through verifying the `.env` file content, clearing browser cache, starting the server in their own terminal, and accessing the website. It also includes troubleshooting steps for persistent errors.
evaluation: |
  The task of instructing the user on how to run the Docusaurus server locally and addressing past issues was completed with detailed guidance.
---