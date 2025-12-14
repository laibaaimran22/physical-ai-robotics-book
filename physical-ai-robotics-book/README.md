# Physical AI and Humanoid Robotics Book

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

<!-- Build Trigger: 2025-01-06 16:45:00 -->

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Backend Configuration

This project requires a backend API server to handle authentication and personalization features. Make sure your FastAPI backend server is running:

```bash
# In your backend directory
uvicorn main:app --reload --port 8000
```

The frontend expects the backend to be available at `http://127.0.0.1:8000` by default. To change this, update the `BACKEND_URL` in `src/config/api.ts`.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
