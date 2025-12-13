# Authentication + Background Personalization System

## Overview
This implementation adds a comprehensive authentication and personalization system to the Physical AI & Humanoid Robotics Book platform. The system collects user background information during signup and uses it to personalize the learning experience.

## Features Implemented

### 1. Enhanced User Profile
- Added fields for software level (beginner/intermediate/advanced)
- Added fields for hardware level (beginner/intermediate/advanced)
- Added field for preferred programming languages
- Added field for learning goals

### 2. Authentication Flows
- **Signup Flow**: Extended signup form with background questions
- **Signin Flow**: Standard email/password authentication
- **Profile Management**: Update and view personalization data

### 3. Personalization Engine
- Difficulty-based content adaptation
- Language-specific examples
- Goal-oriented recommendations
- Experience-level appropriate exercises

### 4. Frontend Components
- **Signup Page**: `/signup` - Extended with background questions
- **Signin Page**: `/signin` - Standard authentication
- **Profile Page**: `/profile` - View and edit background information
- **Personalized Introduction**: Component for chapter personalization
- **Personalization Utilities**: Helper functions for frontend

### 5. Backend API Endpoints
- `GET /api/v1/auth/profile` - Retrieve user profile with background
- `POST /api/v1/auth/profile/update` - Update background information
- `GET /api/v1/auth/personalize/{content_type}` - Get personalized content

### 6. RAG Chatbot Integration
- User background information passed to chatbot prompts
- Personalized responses based on experience level
- Language and goal-specific examples

## Technical Implementation

### Backend Structure
```
backend/
├── src/
│   ├── api/
│   │   ├── auth_background.py    # New auth endpoints
│   ├── services/
│   │   ├── personalization_service.py  # Personalization logic
│   │   └── user_service.py       # User retrieval
│   └── models/
│       └── user.py               # Extended user model
```

### Frontend Structure
```
physical-ai-robotics-book/
├── src/
│   ├── pages/
│   │   ├── signup.tsx           # Extended signup page
│   │   ├── signin.tsx           # Signin page
│   │   └── profile.tsx          # Profile management
│   ├── components/
│   │   └── Personalization/     # Personalization components
│   │       ├── PersonalizedIntroduction.tsx
│   │       └── ...
│   ├── hooks/
│   │   └── useAuth.ts           # Authentication hook
│   └── utils/
│       └── personalization.ts   # Frontend utilities
```

## Database Schema Changes
- Added `software_background_level` column to users table
- Added `hardware_background_level` column to users table
- Added `preferred_languages` column to users table
- Added `learning_goals` column to users table

## Integration Points
- Existing JWT authentication system preserved
- RAG chatbot enhanced with personalization context
- Docusaurus pages can use PersonalizedIntroduction component
- All existing features remain unchanged

## Usage

### For Users
1. Sign up with background information
2. View personalized content on chapter pages
3. Update profile information as needed
4. Receive personalized chatbot responses

### For Developers
1. Use `PersonalizedIntroduction` component in chapter pages
2. Access user background via auth context
3. Leverage personalization utilities for custom implementations

## Security Considerations
- All personalization endpoints require authentication
- User data is stored securely in the database
- Background information is only used for personalization
- JWT tokens continue to secure all API endpoints

## Testing
- Signup flow with background information
- Profile update functionality
- Personalized content rendering
- RAG chatbot personalization
- Authentication state management