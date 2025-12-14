@echo off
echo.
echo ===============================================
echo    Railway Deployment Script for Beginners
echo ===============================================
echo.
echo This script will help you deploy your backend to Railway.
echo.
echo BEFORE RUNNING THIS SCRIPT:
echo 1. Make sure you have Node.js installed
echo 2. Install Railway CLI by running: npm install -g @railway/cli
echo 3. Create a Railway account at: https://railway.app
echo 4. Run 'railway login' in your terminal to log in to Railway
echo.
echo Press any key to continue when you're ready...
pause >nul

cd /d "C:\Users\laiba\OneDrive\Desktop\hackathon-book\backend"

echo.
echo Step 1: Checking if logged into Railway...
railway whoami
if %errorlevel% neq 0 (
    echo ERROR: You are not logged into Railway!
    echo Please run 'railway login' in your terminal first
    pause
    exit /b 1
)

echo.
echo Step 2: Creating/Updating Railway project...
railway up
if %errorlevel% neq 0 (
    echo ERROR during deployment!
    pause
    exit /b 1
)

echo.
echo Step 3: Setting environment variables...
railway var set QDRANT_URL="https://649cf293-7c71-4831-89a5-fda46a3f47cd.us-east4-0.gcp.cloud.qdrant.io"
railway var set QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.xwgQykUtge_6CQ2UTRTDM3KaegYCNOO7Pnm7iAnP_ZQ"
railway var set QDRANT_CLUSTER_ID="649cf293-7c71-4831-89a5-fda46a3f47cd"
railway var set DATABASE_URL="postgresql://neondb_owner:npg_WCvOq9HsUAJ1@ep-misty-morning-ad36vr3e-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require"
railway var set APP_ENV="production"
railway var set APP_HOST="0.0.0.0"
railway var set APP_PORT="8000"
railway var set DEBUG="false"
railway var set GOOGLE_GEMINI_API_KEY="AIzaSyBBh0zXFlmzMBw7-1-jiPvHIotKBG6A6OY"
railway var set JWT_SECRET_KEY="your-super-secret-jwt-key-change-in-production"

echo.
echo Step 4: Redeploying with environment variables...
railway up

echo.
echo Deployment completed!
echo.
echo To view your deployment, run: railway open
echo To get your backend URL, run: railway domain
echo.
echo Press any key to exit...
pause >nul