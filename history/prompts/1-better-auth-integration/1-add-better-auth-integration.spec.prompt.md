---
id: 1
title: Add Better Auth integration
stage: spec
date: 2025-12-11
surface: agent
model: Claude Sonnet 4.5
feature: better-auth-integration
branch: 1-better-auth-integration
user: claude-code
command: /sp.specify
labels: [auth,authentication,better-auth,docusaurus]
links:
  spec: ../specs/1-better-auth-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-better-auth-integration/spec.md
  - specs/1-better-auth-integration/checklists/requirements.md
tests:
  - specs/1-better-auth-integration/checklists/requirements.md
---

## Prompt

I want to add a simple authentication system using Better Auth (https://www.better-auth.com/) to my Docusaurus + FastAPI project. I only want Better Auth self-hosted with FileSystem storage and nothing complex.

Here is what I need:

1. Implement Signup and Signin using Better Auth with FileSystem storage.
2. At signup, ask the user additional questions:
   - Software background level (beginner / intermediate / advanced)
   - Hardware background level (beginner / intermediate / advanced)
   - Optional free-text field: "What do you want to learn?"
3. Store these background fields in the user profile using Better Auth's extra attributes feature.
4. These user background details will later be used to personalize the book content.
5. Make sure authentication integrates cleanly with my existing FastAPI backend.
6. The frontend is a Docusaurus book, so provide:
   - A simple React signup page
   - A simple React signin page
   - A helper function to check whether a user is logged in
7. Do NOT add JWT manually

## Response snapshot

Created a feature specification for integrating Better Auth into the Docusaurus + FastAPI project. The specification includes:

- User scenarios for signup, signin, and session management
- Functional requirements covering authentication, data storage, and integration
- Key entities (User and Authentication Session)
- Measurable success criteria

The specification addresses all requirements including the additional background questions during signup and storing them as user profile attributes.

## Outcome

- ✅ Impact: Created complete feature specification for Better Auth integration
- 🧪 Tests: Specification quality checklist validates all requirements
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase (/sp.plan)
- 🧠 Reflection: Specification covers all user requirements with clear acceptance criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase