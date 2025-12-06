# GitHub Repository Setup Instructions

## Issue
When clicking the GitHub option in the navigation bar or footer while running the site locally, you encounter a "page not found" error. This occurs because the configured GitHub repository (`https://github.com/laibaaimran22/physical-ai-robotics-book`) does not exist yet.

## Solution
To fix this issue, you need to create the GitHub repository and push your local code to it.

### Step 1: Create GitHub Repository
1. Go to [GitHub.com](https://github.com)
2. Click the "+" icon in the top-right corner and select "New repository"
3. Name your repository: `physical-ai-robotics-book`
4. Set the visibility to your preference (public or private)
5. Do NOT initialize with a README, .gitignore, or license (since you already have these locally)
6. Click "Create repository"

### Step 2: Link Your Local Repository to GitHub
From your project directory (`physical-ai-robotics-book`), run:

```bash
git remote add origin https://github.com/laibaaimran22/physical-ai-robotics-book.git
git branch -M main
git push -u origin main
```

### Step 3: Verify the Setup
After pushing your code to GitHub, the GitHub links in your local development site should work correctly and direct users to the actual repository.

## Alternative: Temporarily Disable GitHub Links
If you don't want to create a GitHub repository immediately, you can temporarily modify the links in `docusaurus.config.ts`:

- In the navbar section (line ~92): Change the `href` value from `'https://github.com/laibaaimran22/physical-ai-robotics-book'` to `'#'` or remove the item entirely
- In the footer section (line ~136): Make the same change

## Notes
- The GitHub Pages deployment is already configured in your `docusaurus.config.ts` file
- Once you push to the GitHub repository, you can also enable GitHub Pages in the repository settings for online hosting