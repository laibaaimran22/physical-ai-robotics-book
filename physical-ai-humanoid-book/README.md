---
title: Physical AI & Humanoid Robotics RAG Chatbot
emoji: 🤖
colorFrom: purple
colorTo: yellow
sdk: docker
pinned: false
---

# Physical AI & Humanoid Robotics RAG Chatbot

This Space provides a RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics book. It uses Google's Gemini AI model to answer questions about the content.

## Features

- RAG (Retrieval-Augmented Generation) for accurate responses
- Integration with Google Gemini AI
- Vector database (Qdrant) for efficient document retrieval
- Translation capabilities
- Full-text search of the Physical AI & Humanoid Robotics book

## Usage

The backend provides API endpoints for:
- Chatbot interactions
- Document ingestion
- Content search
- Translation services

## Architecture

- FastAPI backend
- Qdrant vector database
- Google Gemini integration
- PostgreSQL database for user management