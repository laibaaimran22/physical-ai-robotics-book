# Implementation Plan: Urdu Translation Feature

**Branch**: `fix-github-integration-and-update-constitution` | **Date**: 2025-12-09 | **Spec**: [link]
**Input**: Feature specification from `/specs/fix-github-integration-and-update-constitution/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive Urdu Translation Feature for the Docusaurus book platform that allows logged-in users to translate chapter content into Urdu by pressing a "Translate to Urdu" button at the start of each chapter. The system sends page content to the FastAPI backend, uses Google Gemini API to translate into Urdu, returns the translated content, and replaces the chapter content with Urdu text on the frontend. Includes a "Back to English" button to restore original content. The feature works with Docusaurus pages under `/docs/*` and integrates cleanly with existing RAG backend (Neon + Qdrant) without breaking existing chatbot functionality.

## Technical Context

**Language/Version**: Python 3.9+ (backend), TypeScript/React (frontend)
**Primary Dependencies**: FastAPI, Google Gemini API, React, Docusaurus, Qdrant, Neon Postgres
**Storage**: N/A (translation is stateless, though translation history could be stored)
**Testing**: pytest (backend), React testing library (frontend)
**Target Platform**: Web application (Docusaurus + FastAPI backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Translation response under 5 seconds, API response times under 2 seconds for 95th percentile
**Constraints**: <200ms p95 for internal API calls, must not break existing RAG/chatbot functionality, must work with Docusaurus MDX pages
**Scale/Scope**: Support 1000+ concurrent users with translation capability

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Backend Architecture Excellence (Constitution Section V)**: ✅ The translation feature will be implemented using FastAPI following the existing architecture patterns with async operations and proper error handling.

2. **System-wide Requirements (Constitution Section A)**: ✅ The translation API will use the same backend stack (FastAPI, async architecture) and integrate with existing systems without breaking functionality.

3. **Architecture Principles (Constitution Section 1-5)**: ✅ The implementation will follow:
   - Async-First Design: Translation endpoint will be asynchronous
   - Separation of Concerns: Translation service will be separate from other services
   - Type Safety: Pydantic models for request/response validation
   - Security-First: Proper authentication and rate limiting
   - Observability: Proper logging for debugging

4. **API Requirements (Constitution Section)**: ✅ The translation API will follow the same patterns as existing APIs with proper request/response validation.

5. **Resource Optimization (Constitution Section D)**: ✅ Translation API will include rate limiting and resource management to prevent abuse.

## Project Structure

### Documentation (this feature)

```text
specs/fix-github-integration-and-update-constitution/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   │   └── translation.py    # New translation API endpoints
│   └── config/
│       └── settings.py       # Updated with translation settings
└── tests/

frontend/
physical-ai-robotics-book/
├── src/
│   ├── components/
│   │   ├── TranslationButton/
│   │   │   ├── TranslationButton.tsx      # Translation button component
│   │   │   └── TranslationButton.module.css  # Styling for translation button
│   │   ├── TranslationWrapper/
│   │   │   ├── TranslationWrapper.tsx     # Content wrapper component
│   │   │   └── TranslationWrapper.module.css # Styling for wrapper
│   │   └── Chatbot/
│   ├── hooks/
│   │   └── useTranslation.ts              # Translation state management hook
│   └── pages/
└── tests/
```

**Structure Decision**: Web application structure with backend API for translation and frontend React components for user interface. The backend follows the existing FastAPI architecture with new translation endpoints, while the frontend implements React components and hooks for translation functionality that integrates with Docusaurus MDX pages.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Additional API endpoint | Required for translation functionality | Could have used client-side translation but lacks accuracy and requires external API access |
| New frontend components | Required for user interface | Could have modified existing components but translation requires specific UI elements |
