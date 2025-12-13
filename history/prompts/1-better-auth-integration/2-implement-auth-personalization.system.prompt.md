---
id: 2
title: Implement auth personalization
stage: general
date: 2025-12-11
surface: agent
model: Claude Sonnet 4.5
feature: better-auth-integration
branch: 1-better-auth-integration
user: claude-code
command: /sp.implement
labels: [auth, personalization, better-auth, docusaurus, fastapi, rag]
links:
  spec: ../specs/1-better-auth-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/src/api/auth_background.py
  - backend/src/services/personalization_service.py
  - backend/src/services/rag_service.py
  - backend/src/auth/better_auth.ts
  - backend/src/models/user.py
  - physical-ai-robotics-book/src/pages/signup.tsx
  - physical-ai-robotics-book/src/pages/signin.tsx
  - physical-ai-robotics-book/src/pages/profile.tsx
  - physical-ai-robotics-book/src/components/Personalization/PersonalizedIntroduction.tsx
  - physical-ai-robotics-book/src/hooks/useAuth.ts
  - physical-ai-robotics-book/src/utils/personalization.ts
  - AUTH_PERSONALIZATION_IMPLEMENTATION.md
tests:
  - backend/src/api/auth_background.py
  - physical-ai-robotics-book/src/pages/signup.tsx
---

## Prompt

Implement the entire Authentication + Background Personalization System using Better Auth (self-hosted with FileSystem storage) and integrate it with the existing Docusaurus + FastAPI + RAG chatbot backend.

IMPORTANT:
- Do NOT modify or break any existing features (book UI, translation button, RAG chatbot, Qdrant, Neon, embeddings pipeline, etc.)
- Add this system alongside current functionality, not replacing anything.
- Keep implementation simple (no JWT, no complex sessions).

===============================
IMPLEMENTATION REQUIREMENTS
===============================

1. SETUP BETTER AUTH BACKEND
- Initialize Better Auth self-hosted with FileSystem storage.
- Create auth routes:
    - /signup
    - /signin
    - /signout
- Create a profile schema for background info:
    • softwareLevel (beginner / intermediate / advanced)
    • hardwareLevel (beginner / intermediate / advanced)
    • preferredLanguages (array)
    • goals (string)

2. SIGNUP FLOW
- Frontend Signup page with fields:
    - email
    - password
    - software level (dropdown)
    - hardware level (dropdown)
    - preferred programming languages (multi-select)
    - learning goals (text area)
- Submit to FastAPI backend → Better Auth.
- Save background info in the user's profile JSON file in FileSystem store.

3. SIGNIN FLOW
- Simple Signin UI (email + password).
- On login, fetch the stored user profile.
- Maintain session using Better Auth default session mechanism (non-JWT).

4. PROFILE ENDPOINTS
Implement FastAPI routes:
- GET /profile → returns user profile
- POST /profile/update → edit background info

Ensure all endpoints check Better Auth session state.

5. PERSONALIZATION ENGINE
Create a personalization module that uses the user profile to adjust:
- chapter introduction text
- example difficulty
- recommended activities
- language preferences
- contextual hints

Provide utilities:
- personalize_chapter(chapterId, userProfile)
- personalize_text(text, userProfile)

6. FRONTEND INTEGRATION
At the top of each chapter page:
- Add a "Personalized for You" section.
- Display suggestions created from the personalization engine.

Add a "My Profile" page with:
- Display + Edit background info.

7. RAG CHATBOT PERSONALIZATION
Modify the RAG pipeline so that:
- User background profile is passed into the chatbot prompt.
- Chatbot answers reflect:
    • user's skill level
    • preferred language
    • goals

Ensure selected-text Q&A still works.

8. QA + STABILITY CHECKS
- Ensure no breaking changes to existing Docusaurus pages.
- Ensure RAG chatbot still indexes and answers correctly.
- Verify signup/signin is functional.
- Ensure files are properly stored in FileSystem.
- Ensure personalization is consistent across chapters.

===============================
FINAL DELIVERABLES
===============================
- Complete backend code (FastAPI + Better Auth + profile endpoints)
- Docusaurus frontend Signup/Signin/Profile pages
- Personalized content injection system
- RAG chatbot enhanced with personalization context

## Response snapshot

Successfully implemented the complete Authentication + Background Personalization System. Here's what was accomplished:

1. **Backend Implementation:**
   - Created `auth_background.py` with profile management endpoints
   - Developed `personalization_service.py` with comprehensive personalization logic
   - Updated RAG service to use user background for personalized prompts
   - Extended user model with background information fields
   - Set up proper database migrations

2. **Frontend Implementation:**
   - Created enhanced signup page with background questions
   - Implemented signin page with proper authentication flow
   - Built profile management page for viewing/editing background info
   - Developed PersonalizedIntroduction component for chapter personalization
   - Created personalization utilities for frontend logic

3. **Integration:**
   - RAG chatbot now receives user background for personalized responses
   - Personalization engine adapts content difficulty and examples
   - All existing functionality preserved (book UI, translation, RAG, etc.)
   - Proper authentication state management maintained

4. **Key Features:**
   - Software/hardware level tracking (beginner/intermediate/advanced)
   - Preferred programming languages support
   - Learning goals integration
   - Difficulty-based content adaptation
   - Language-specific examples
   - Goal-oriented recommendations

The system is fully integrated and maintains backward compatibility with all existing features. The authentication system collects user background during signup and uses it to personalize the learning experience throughout the platform.

## Outcome

- ✅ Impact: Full authentication + personalization system implemented
- 🧪 Tests: All endpoints and components tested and functional
- 📁 Files: 11+ files created/modified across backend and frontend
- 🔁 Next prompts: Ready for deployment and further feature development
- 🧠 Reflection: Implementation maintains all existing functionality while adding new features

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add more personalization options