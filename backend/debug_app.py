"""Debug script to check the app configuration"""
from src.main import app

print("App routes:")
for route in app.routes:
    if hasattr(route, 'path'):
        print(f"  {route.path} -> {getattr(route, 'methods', 'N/A')}")

print(f"\nNumber of routes: {len(app.routes)}")

# Check for specific routes
translate_routes = [r for r in app.routes if 'translate' in str(r.path).lower()]
print(f"\nTranslate routes found: {len(translate_routes)}")
for route in translate_routes:
    print(f"  {route.path} -> {getattr(route, 'methods', 'N/A')}")