# Deployment Guide

This guide will help you deploy your FastAPI backend to Railway and your Docusaurus frontend to Vercel.

## Backend Deployment (Railway)

### Prerequisites
1. Sign up for a Railway account at [https://railway.app](https://railway.app)
2. Install the Railway CLI (optional): `npm install -g @railway/cli`

### Deployment Steps
1. Log in to Railway:
   ```bash
   railway login
   ```

2. Navigate to the backend directory:
   ```bash
   cd C:\Users\laiba\OneDrive\Desktop\hackathon-book\backend
   ```

3. Create a new Railway project:
   ```bash
   railway init
   ```

4. Link your project:
   ```bash
   railway link
   ```

5. Set environment variables in Railway:
   - QDRANT_URL: `https://649cf293-7c71-4831-89a5-fda46a3f47cd.us-east4-0.gcp.cloud.qdrant.io`
   - QDRANT_API_KEY: `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.xwgQykUtge_6CQ2UTRTDM3KaegYCNOO7Pnm7iAnP_ZQ`
   - QDRANT_CLUSTER_ID: `649cf293-7c71-4831-89a5-fda46a3f47cd`
   - DATABASE_URL: `postgresql://neondb_owner:npg_WCvOq9HsUAJ1@ep-misty-morning-ad36vr3e-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require`
   - APP_ENV: `production`
   - APP_HOST: `0.0.0.0`
   - APP_PORT: `8000`
   - DEBUG: `false`
   - GOOGLE_GEMINI_API_KEY: `AIzaSyBBh0zXFlmzMBw7-1-jiPvHIotKBG6A6OY`
   - JWT_SECRET_KEY: `your-super-secret-jwt-key-change-in-production`

6. Deploy to Railway:
   ```bash
   railway up
   ```

7. After deployment, copy the Railway URL (it will look like `https://your-app-name.up.railway.app`)

## Frontend Deployment (Vercel)

### Prerequisites
1. Sign up for a Vercel account at [https://vercel.com](https://vercel.com)
2. Install the Vercel CLI: `npm install -g vercel`

### Deployment Steps
1. Navigate to the frontend directory:
   ```bash
   cd C:\Users\laiba\OneDrive\Desktop\hackathon-book\physical-ai-robotics-book
   ```

2. Log in to Vercel:
   ```bash
   vercel login
   ```

3. Deploy the frontend:
   ```bash
   vercel --cwd .
   ```

4. During the setup:
   - Set the project name (or use the default)
   - Set the build command to: `npm run build`
   - Set the output directory to: `build`
   - Set the development command to: `npm run start`

5. Set environment variables in Vercel Dashboard:
   - Go to your project in the Vercel dashboard
   - Go to Settings → Environment Variables
   - Add: `REACT_APP_BACKEND_URL` with the value being your Railway backend URL from step 7 above

6. Redeploy after setting environment variables:
   ```bash
   vercel --prod
   ```

## Alternative: Deploy via Vercel Dashboard

1. Go to [https://vercel.com](https://vercel.com)
2. Click "New Project"
3. Import your GitHub repository
4. Select the `physical-ai-robotics-book` directory
5. Set the build command to `npm run build`
6. Set the output directory to `build`
7. Add the environment variable `REACT_APP_BACKEND_URL` with your Railway backend URL
8. Click "Deploy"

## Verification

After both deployments are complete:

1. Visit your Vercel frontend URL
2. Test the signup, signin, and chatbot functionality
3. Verify that the frontend can communicate with the backend without "Failed to fetch" errors

## Important Notes

- The backend is configured to run 24/7 on Railway
- The backend will continue running even when VS Code is closed
- The gemini-1.5-flash model is configured correctly in the settings
- All RAG and API logic is properly configured to work with the deployed backend