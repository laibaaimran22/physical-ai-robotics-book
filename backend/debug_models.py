#!/usr/bin/env python3
"""Debug script to check model imports and metadata."""

import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def test_model_imports():
    """Test model imports and metadata registration."""
    print("Testing model imports and metadata registration...")

    # Try importing the models module first
    print("1. Importing models module...")
    try:
        import src.models
        print("   SUCCESS: Models module imported successfully")
    except Exception as e:
        print(f"   ERROR: Failed to import models module: {e}")
        return False

    # Check Base metadata after importing models
    print("2. Checking Base metadata after importing models...")
    from src.database.base import Base
    table_names = [table.name for table in Base.metadata.tables.values()]
    print(f"   Tables in Base metadata: {table_names}")

    if not table_names:
        print("   ERROR: No tables found in Base metadata")

        # Try importing models directly
        print("3. Importing models directly...")
        try:
            from src.models.user import User
            from src.models.rag_query import RAGQuery
            from src.models.chat_history import ChatHistory
            print("   SUCCESS: Individual models imported successfully")

            # Check Base metadata again
            table_names = [table.name for table in Base.metadata.tables.values()]
            print(f"   Tables in Base metadata after direct import: {table_names}")

            if table_names:
                print("   SUCCESS: Tables now registered in Base metadata!")
                return True
            else:
                print("   ERROR: Still no tables in Base metadata after direct import")
                return False
        except Exception as e:
            print(f"   ERROR: Failed to import individual models: {e}")
            return False
    else:
        print("   SUCCESS: Tables found in Base metadata!")
        return True

if __name__ == "__main__":
    success = test_model_imports()
    print(f"\nModel import test: {'SUCCESS' if success else 'FAILED'}")