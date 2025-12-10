# Implementation Tasks: Urdu Translation Feature

**Feature**: Urdu Translation Feature for Docusaurus Book Platform
**Branch**: `fix-github-integration-and-update-constitution`
**Date**: 2025-12-09
**Input**: `/specs/fix-github-integration-and-update-constitution/spec.md` and `plan.md`

## Feature Overview

Implementation of a comprehensive Urdu Translation Feature for the Docusaurus book platform that allows logged-in users to translate chapter content into Urdu by pressing a "Translate to Urdu" button at the start of each chapter. The system sends page content to the FastAPI backend, uses Google Gemini API to translate into Urdu, returns the translated content, and replaces the chapter content with Urdu text on the frontend. Includes a "Back to English" button to restore original content. The feature works with Docusaurus pages under `/docs/*` and integrates cleanly with existing RAG backend (Neon + Qdrant) without breaking existing chatbot functionality.

## Implementation Strategy

MVP approach: Start with core translation functionality (backend API + basic frontend components), then add advanced features like state management, styling, and integration with existing components.

## Dependencies

- Google Gemini API (for translation)
- FastAPI backend
- Docusaurus frontend
- React/TypeScript
- Existing RAG system (Neon Postgres + Qdrant)

## Parallel Execution Opportunities

- Backend API development can run in parallel with frontend component development
- Translation button and wrapper components can be developed independently
- CSS styling can be done in parallel with component development

---

## Phase 1: Setup Tasks

**Goal**: Initialize project structure and dependencies for the Urdu Translation feature

- [X] T084 Set up backend translation API structure in `backend/src/api/`
- [X] T085 Install Google Gemini API dependencies in `backend/requirements.txt`
- [X] T086 Create frontend components directory structure in `physical-ai-robotics-book/src/components/TranslationButton/`
- [X] T087 Create frontend hooks directory structure in `physical-ai-robotics-book/src/hooks/`
- [X] T088 Add GEMINI_API_KEY to settings configuration in `backend/src/config/settings.py`
- [X] T089 Update CORS configuration to allow translation API requests

## Phase 2: Foundational Tasks

**Goal**: Implement core infrastructure needed by all user stories

- [X] T090 Create TranslationRequest Pydantic model in `backend/src/models/translation.py`
- [X] T091 Create TranslationResponse Pydantic model in `backend/src/models/translation.py`
- [X] T092 Implement Google Gemini API client in `backend/src/core/gemini_client.py`
- [X] T093 Create translation service in `backend/src/services/translation_service.py`
- [X] T094 Add translation endpoint to main FastAPI app in `backend/src/main.py`
- [X] T095 [P] Create CSS module for translation button in `physical-ai-robotics-book/src/components/TranslationButton/TranslationButton.module.css`
- [X] T096 [P] Create CSS module for translation wrapper in `physical-ai-robotics-book/src/components/TranslationWrapper/TranslationWrapper.module.css`

## Phase 3: [US1] Basic Translation API

**Goal**: Implement backend translation API that accepts content and returns Urdu translation

**Independent Test Criteria**: Translation endpoint successfully converts English text to Urdu using Google Gemini API

- [X] T097 [US1] Implement translation API endpoint in `backend/src/api/translation.py`
- [X] T098 [US1] Add error handling for translation API in `backend/src/api/translation.py`
- [X] T099 [US1] Implement rate limiting for translation API in `backend/src/api/translation.py`
- [X] T100 [US1] Add API key validation for translation endpoint in `backend/src/api/translation.py`
- [X] T101 [US1] Test translation API with sample content
- [X] T102 [US1] Add logging for translation requests in `backend/src/api/translation.py`

## Phase 4: [US2] Translation Button Component

**Goal**: Create frontend button component that triggers translation functionality

**Independent Test Criteria**: Translation button appears on page and triggers translation when clicked

- [X] T103 [US2] Create TranslationButton React component in `physical-ai-robotics-book/src/components/TranslationButton/TranslationButton.tsx`
- [X] T104 [US2] Add loading state to TranslationButton component
- [X] T105 [US2] Add error handling to TranslationButton component
- [X] T106 [US2] Style TranslationButton with CSS module
- [X] T107 [US2] Add accessibility attributes to TranslationButton
- [X] T108 [US2] Test TranslationButton functionality in isolation

## Phase 5: [US3] Translation State Management

**Goal**: Implement React hook for managing translation state and API communication

**Independent Test Criteria**: Translation state hook can manage original content, translated content, and loading states

- [X] T109 [US3] Create useTranslation hook in `physical-ai-robotics-book/src/hooks/useTranslation.ts`
- [X] T110 [US3] Implement translation API call in useTranslation hook
- [X] T111 [US3] Add state management for loading, error, and success states in useTranslation hook
- [X] T112 [US3] Implement content switching logic in useTranslation hook
- [X] T113 [US3] Add error recovery functionality in useTranslation hook
- [X] T114 [US3] Test useTranslation hook functionality

## Phase 6: [US4] Translation Wrapper Component

**Goal**: Create wrapper component that manages content translation and integrates with Docusaurus pages

**Independent Test Criteria**: Translation wrapper can wrap content and toggle between English and Urdu versions

- [X] T115 [US4] Create TranslationWrapper React component in `physical-ai-robotics-book/src/components/TranslationWrapper/TranslationWrapper.tsx`
- [X] T116 [US4] Integrate useTranslation hook with TranslationWrapper
- [X] T117 [US4] Add content extraction functionality to TranslationWrapper
- [X] T118 [US4] Implement content display logic in TranslationWrapper
- [X] T119 [US4] Add back-to-English button to TranslationWrapper
- [X] T120 [US4] Style TranslationWrapper with CSS module
- [X] T121 [US4] Test TranslationWrapper integration with sample content

## Phase 7: [US5] Docusaurus Integration

**Goal**: Integrate translation components with Docusaurus MDX pages

**Independent Test Criteria**: Translation components work seamlessly with Docusaurus documentation pages

- [ ] T122 [US5] Create Docusaurus plugin configuration for translation components
- [ ] T123 [US5] Add translation button to Docusaurus page layout
- [ ] T124 [US5] Implement automatic detection of content sections for translation
- [ ] T125 [US5] Test translation functionality on sample Docusaurus pages
- [ ] T126 [US5] Add translation functionality to all documentation pages
- [ ] T127 [US5] Ensure translation state persists across page navigation

## Phase 8: [US6] Advanced Translation Features

**Goal**: Add advanced features like translation history, caching, and enhanced UX

**Independent Test Criteria**: Advanced translation features work correctly and improve user experience

- [ ] T128 [US6] Implement translation caching to avoid repeated API calls
- [ ] T129 [US6] Add progress indicators for long translation requests
- [ ] T130 [US6] Implement translation history tracking
- [ ] T131 [US6] Add RTL (right-to-left) styling for Urdu text
- [ ] T132 [US6] Implement keyboard shortcuts for translation toggle
- [ ] T133 [US6] Add translation quality feedback mechanism

## Phase 9: Polish & Cross-Cutting Concerns

**Goal**: Finalize implementation with testing, documentation, and optimization

- [ ] T134 Add comprehensive tests for translation API endpoints
- [ ] T135 Add unit tests for frontend translation components
- [ ] T136 Update API documentation with translation endpoints
- [ ] T137 Add error handling and validation for edge cases
- [ ] T138 Optimize translation API performance and caching
- [ ] T139 Add monitoring and logging for translation usage
- [ ] T140 Update user documentation with translation feature instructions
- [ ] T141 Perform security review of translation API
- [ ] T142 Test translation feature with different content types and lengths
- [ ] T143 Final integration testing with existing RAG functionality

---

## Dependencies

- User Story 3 (useTranslation hook) must be completed before User Story 4 (TranslationWrapper)
- User Story 1 (Translation API) must be completed before User Story 2 (TranslationButton)
- User Story 4 (TranslationWrapper) must be completed before User Story 5 (Docusaurus Integration)

## Parallel Execution Examples

- Tasks T095 and T096 can run in parallel (different CSS files)
- User Story 2 (TranslationButton) and User Story 3 (useTranslation) can run in parallel (frontend/backend)
- User Story 6 (Advanced Features) can run after User Story 4 (TranslationWrapper) is complete

## MVP Scope

The minimum viable product includes:
- Translation API (T097-T102)
- TranslationButton component (T103-T108)
- useTranslation hook (T109-T114)
- TranslationWrapper component (T115-T121)
- Basic Docusaurus integration (T122-T125)

## Test Scenarios for Each User Story

### User Story 1: Basic Translation API
- Translation API successfully accepts English content and returns Urdu translation
- API handles errors gracefully when Gemini API is unavailable
- Rate limiting works correctly for translation requests
- API validates input content length and format

### User Story 2: Translation Button Component
- Translation button renders correctly on Docusaurus pages
- Button changes state during translation process
- Button displays appropriate loading and error states
- Button is accessible via keyboard navigation

### User Story 3: Translation State Management
- useTranslation hook manages original and translated content correctly
- Hook handles API errors and provides appropriate feedback
- State persists correctly during translation process
- Hook allows switching between English and Urdu content

### User Story 4: Translation Wrapper Component
- TranslationWrapper correctly extracts content from Docusaurus pages
- Component displays translated content properly
- Back-to-English functionality works correctly
- Component handles different content types and lengths

### User Story 5: Docusaurus Integration
- Translation functionality works across all Docusaurus documentation pages
- Translation state persists during page navigation
- Integration doesn't interfere with existing Docusaurus functionality
- Components work with different Docusaurus themes and layouts

### User Story 6: Advanced Translation Features
- Translation caching reduces API calls for repeated content
- Progress indicators provide user feedback during long translations
- RTL styling displays Urdu text correctly
- Keyboard shortcuts provide convenient access to translation features