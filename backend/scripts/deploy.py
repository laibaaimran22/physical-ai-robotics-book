#!/usr/bin/env python3
"""
Deployment script for the Physical AI & Humanoid Robotics Book Platform Backend
"""
import os
import sys
import subprocess
from pathlib import Path


def run_command(command: str, cwd: str = None) -> bool:
    """
    Run a shell command and return True if successful, False otherwise.
    """
    try:
        print(f"Running: {command}")
        result = subprocess.run(
            command,
            shell=True,
            cwd=cwd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        print(f"stderr: {e.stderr}")
        return False


def main():
    """
    Main deployment function.
    """
    print("Starting deployment for Physical AI & Humanoid Robotics Book Platform Backend...")

    # Check if we're in the correct directory
    current_dir = Path.cwd()
    if not (current_dir / "backend" / "src").exists():
        print("Error: Not in the project root directory")
        sys.exit(1)

    backend_dir = current_dir / "backend"

    # Install dependencies
    print("\nInstalling dependencies...")
    if not run_command("pip install -r requirements.txt", cwd=str(backend_dir)):
        print("Failed to install dependencies")
        sys.exit(1)

    # Run database migrations (if any exist)
    print("\nRunning database migrations...")
    # For now, we'll just show a message since we haven't created the migration system yet
    print("Note: Database migrations would be run here in a production deployment")

    # Build/compile assets if needed
    print("\nBuilding assets...")
    # Currently no build process needed for this FastAPI app

    # Run tests (if available)
    print("\nRunning tests...")
    # For now, just a message since we haven't created test files
    print("Note: Tests would be run here in a production deployment")

    # Start the application
    print("\nStarting the application...")
    print("Use the following command to start the server:")
    print(f"cd {backend_dir} && uvicorn src.main:app --host 0.0.0.0 --port 8000")

    print("\nDeployment completed successfully!")


if __name__ == "__main__":
    main()