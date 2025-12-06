---
id: 1
title: "Fix Tsconfig Mdx Errors"
stage: "general"
date: "2025-12-06"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-physical-ai-book"
user: "laiba"
command: "fix-tsconfig-mdx-errors"
labels: ["typescript", "mdx", "compilation", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "physical-ai-robotics-book/tsconfig.json"
  - "physical-ai-robotics-book/docs/module4-vision-language-action/lesson4-2-cognitive-planning-llms.md"
tests: []
prompt: |
  solve the error in tsconfig.json file
  i am having this error Option 'baseUrl' is deprecated and will stop functioning in TypeScript 7.0. Specify compilerOption '"ignoreDeprecations": "6.0"' to silence this error.
  please run the project on localhost now
  it says this program crashed
response: |
  I've successfully fixed the TypeScript and MDX compilation errors and got the Docusaurus development server running on localhost. Here's what was done:

  1. Fixed the tsconfig.json deprecation warning by adding "ignoreDeprecations": "6.0" to compilerOptions
  2. Resolved multiple MDX compilation errors in lesson4-2-cognitive-planning-llms.md:
     - Fixed triple quotes in markdown heading that was causing parsing errors
     - Removed binary characters corrupting ASCII art
     - Fixed nested code block issue where ```json inside ```python was breaking parsing
  3. The development server is now running successfully at http://localhost:3000/your-repo/

  All MDX compilation errors have been resolved and the project builds successfully.
outcome: "Successfully fixed TypeScript deprecation warning and multiple MDX compilation errors, enabling the Docusaurus development server to run without crashes."
evaluation: "The fixes were successful - no more compilation errors, server runs properly"
---

# Fix Tsconfig Mdx Errors

## Prompt
solve the error in tsconfig.json file
i am having this error Option 'baseUrl' is deprecated and will stop functioning in TypeScript 7.0. Specify compilerOption '"ignoreDeprecations": "6.0"' to silence this error.
please run the project on localhost now
it says this program crashed

## Response
I've successfully fixed the TypeScript and MDX compilation errors and got the Docusaurus development server running on localhost. Here's what was done:

1. Fixed the tsconfig.json deprecation warning by adding "ignoreDeprecations": "6.0" to compilerOptions
2. Resolved multiple MDX compilation errors in lesson4-2-cognitive-planning-llms.md:
   - Fixed triple quotes in markdown heading that was causing parsing errors
   - Removed binary characters corrupting ASCII art
   - Fixed nested code block issue where ```json inside ```python was breaking parsing
3. The development server is now running successfully at http://localhost:3000/your-repo/

All MDX compilation errors have been resolved and the project builds successfully.

## Outcome
Successfully fixed TypeScript deprecation warning and multiple MDX compilation errors, enabling the Docusaurus development server to run without crashes.

## Evaluation
The fixes were successful - no more compilation errors, server runs properly