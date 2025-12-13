import os
import re
from pathlib import Path

def get_chapter_id_and_title(file_path):
    """Extract title from the markdown file and generate a chapter ID."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract title from the frontmatter
    title_match = re.search(r'title:\s*["\']([^"\']*)["\']', content)
    if title_match:
        title = title_match.group(1)
        # Generate chapter ID from title
        chapter_id = title.lower().replace(' ', '-').replace('_', '-').replace('"', '').replace("'", "")
        chapter_id = re.sub(r'[^a-z0-9-]', '', chapter_id)
        return chapter_id, title

    return None, None

def update_chapter_file(file_path):
    """Add personalization component to a chapter file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check if personalization component is already added
    if 'ChapterPersonalization' in content:
        print(f"Skipping {file_path} - already has personalization component")
        return False

    # Find the location to insert the personalization component
    # Look for import statements and the opening <ChapterTranslator>
    import_match = re.search(r"import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';", content)
    if not import_match:
        print(f"Skipping {file_path} - no ChapterTranslator import found")
        return False

    # Add the import for ChapterPersonalization
    new_content = content[:import_match.end()] + "\nimport ChapterPersonalization from '@site/src/components/Personalization/ChapterPersonalization';" + content[import_match.end():]

    # Find <ChapterTranslator> tag and insert the personalization component after it
    translator_match = re.search(r'<ChapterTranslator>\s*\n', new_content)
    if translator_match:
        chapter_id, chapter_title = get_chapter_id_and_title(file_path)
        if chapter_id and chapter_title:
            personalization_component = f'<ChapterPersonalization\n  chapterId="{chapter_id}"\n  chapterTitle="{chapter_title}"\n/>\n'
            new_content = new_content[:translator_match.end()] + personalization_component + new_content[translator_match.end():]

            # Write the updated content back to the file
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(new_content)

            print(f"Updated {file_path} with personalization component")
            return True
        else:
            print(f"Could not extract title from {file_path}")
            return False
    else:
        print(f"Could not find <ChapterTranslator> tag in {file_path}")
        return False

def main():
    base_path = "C:\\Users\\laiba\\OneDrive\\Desktop\\hackathon-book\\physical-ai-robotics-book\\docs"

    # Find all markdown files except the node_modules ones
    md_files = []
    for root, dirs, files in os.walk(base_path):
        # Skip node_modules directory
        dirs[:] = [d for d in dirs if d != 'node_modules']
        for file in files:
            if file.endswith('.md') and 'personalization-example' not in file:
                md_files.append(os.path.join(root, file))

    updated_count = 0
    for file_path in md_files:
        if update_chapter_file(file_path):
            updated_count += 1

    print(f"\nSummary: Updated {updated_count} chapter files with personalization components")
    print(f"Total chapter files processed: {len(md_files)}")

if __name__ == "__main__":
    main()