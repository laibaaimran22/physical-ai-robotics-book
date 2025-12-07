#!/usr/bin/env python3
"""
Migration runner for the Physical AI & Humanoid Robotics Book Platform Backend
"""
import asyncio
import sys
from pathlib import Path
from sqlalchemy import text

# Add the backend/src directory to the Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.config.database import sync_engine


def run_migrations():
    """
    Run database migrations.
    This is a simplified migration runner - in a real application, you would use Alembic.
    """
    print("Running database migrations...")

    try:
        # Connect to the database
        with sync_engine.connect() as conn:
            # Create tables based on SQLAlchemy models
            from src.database.base import Base
            Base.metadata.create_all(sync_engine)

            print("Tables created successfully!")

            # Check if required tables exist
            result = conn.execute(text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
            """))

            tables = [row[0] for row in result.fetchall()]
            print(f"Existing tables: {tables}")

            # Create initial admin user if not exists
            result = conn.execute(text("SELECT COUNT(*) FROM users WHERE email = :email"), {"email": "admin@example.com"})
            admin_exists = result.fetchone()[0]

            if not admin_exists:
                conn.execute(text("""
                    INSERT INTO users (email, hashed_password, full_name, is_active, is_admin, created_at, updated_at)
                    VALUES (:email, :password, :full_name, :is_active, :is_admin, NOW(), NOW())
                """), {
                    "email": "admin@example.com",
                    "password": "$2b$12$VcCDgh2.K/cMmIRZu.7Ge.eu8OWoch/BjwjW2SbrGBxe9GqIrS8U2",  # bcrypt hash for "admin123"
                    "full_name": "Admin User",
                    "is_active": True,
                    "is_admin": True
                })
                conn.commit()
                print("Initial admin user created (email: admin@example.com, password: admin123)")
            else:
                print("Admin user already exists")

    except Exception as e:
        print(f"Error running migrations: {e}")
        return False

    print("Migrations completed successfully!")
    return True


def rollback_migrations():
    """
    Rollback database migrations.
    This is a simplified rollback function.
    """
    print("Rolling back migrations...")

    try:
        with sync_engine.connect() as conn:
            # Drop all tables (be careful!)
            from src.database.base import Base
            Base.metadata.drop_all(sync_engine)
            print("All tables dropped successfully!")
    except Exception as e:
        print(f"Error rolling back migrations: {e}")
        return False

    print("Rollback completed successfully!")
    return True


def main():
    """
    Main function to handle migration commands.
    """
    if len(sys.argv) < 2:
        print("Usage: python run_migrations.py [upgrade|downgrade]")
        print("  upgrade: Run migrations to create/update tables")
        print("  downgrade: Rollback migrations to drop all tables")
        sys.exit(1)

    command = sys.argv[1].lower()

    if command == "upgrade":
        run_migrations()
    elif command == "downgrade":
        rollback_migrations()
    else:
        print(f"Unknown command: {command}")
        print("Usage: python run_migrations.py [upgrade|downgrade]")
        sys.exit(1)


if __name__ == "__main__":
    main()