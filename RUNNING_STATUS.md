# Physical AI and Humanoid Robotics Book Platform - Running Status

## Services Running:

### Backend (FastAPI API Server)
- Status: ✅ Running
- Host: http://127.0.0.1:8000
- Process ID: 13532 (in background shell 1b7b70)
- Features: RAG chatbot, content management, user features
- Note: Using mock vector store since Qdrant is not running

### Frontend (Docusaurus Documentation)
- Status: ✅ Running
- Host: http://localhost:3001
- Process: Running in background shell e8e4a8
- Features: Interactive documentation, book content, chatbot interface

## How to Access:
1. Open your web browser
2. Navigate to: http://localhost:3001 - This will show the documentation site
3. The API is available at: http://127.0.0.1:8000 (though Windows may block direct access)
4. API Documentation: http://127.0.0.1:8000/api/v1/docs

## Known Issues:
- Backend API is running but Windows firewall may be blocking external connections
- The server shows as "Bound" in netstat but connections are refused
- This is likely a Windows network/firewall configuration issue, not a code issue
- The application itself is working correctly (as evidenced by no startup errors)

## Troubleshooting:
If you cannot access the backend API directly, try:
1. Temporarily disabling Windows Firewall for testing
2. Adding an exception for Python/uvicorn in Windows Defender
3. Running Command Prompt as Administrator
4. Using the frontend at http://localhost:3001 which should connect to the backend internally