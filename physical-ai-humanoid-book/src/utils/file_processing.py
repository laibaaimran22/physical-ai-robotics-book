import os
import tempfile
from typing import Optional, List, Tuple
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import pypdf


def validate_file_type(file_path: str) -> bool:
    """Validate if the file type is supported for ingestion."""
    supported_extensions = {'.md', '.mdx', '.html', '.htm', '.txt', '.pdf'}
    file_ext = Path(file_path).suffix.lower()
    return file_ext in supported_extensions


def extract_text_from_markdown(file_path: str) -> str:
    """Extract text content from a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
        # Convert markdown to HTML then extract text
        html = markdown.markdown(content)
        soup = BeautifulSoup(html, 'html.parser')
        return soup.get_text()


def extract_text_from_html(file_path: str) -> str:
    """Extract text content from an HTML file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
        soup = BeautifulSoup(content, 'html.parser')
        return soup.get_text()


def extract_text_from_txt(file_path: str) -> str:
    """Extract text content from a plain text file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        return file.read()


def extract_text_from_pdf(file_path: str) -> str:
    """Extract text content from a PDF file."""
    text = ""
    with open(file_path, 'rb') as file:
        pdf_reader = pypdf.PdfReader(file)
        for page in pdf_reader.pages:
            text += page.extract_text() + "\n"
    return text


def extract_text_from_file(file_path: str) -> str:
    """Extract text content from a file based on its extension."""
    if not validate_file_type(file_path):
        raise ValueError(f"Unsupported file type: {file_path}")

    file_ext = Path(file_path).suffix.lower()

    if file_ext in ['.md', '.mdx']:
        return extract_text_from_markdown(file_path)
    elif file_ext in ['.html', '.htm']:
        return extract_text_from_html(file_path)
    elif file_ext == '.txt':
        return extract_text_from_txt(file_path)
    elif file_ext == '.pdf':
        return extract_text_from_pdf(file_path)
    else:
        raise ValueError(f"Unsupported file type: {file_ext}")


def save_uploaded_file(file_content: bytes, upload_dir: str, filename: str) -> str:
    """Save uploaded file to the specified directory."""
    os.makedirs(upload_dir, exist_ok=True)
    file_path = os.path.join(upload_dir, filename)

    with open(file_path, 'wb') as f:
        f.write(file_content)

    return file_path


def get_file_type(file_path: str) -> str:
    """Get the content type based on file extension."""
    file_ext = Path(file_path).suffix.lower()

    if file_ext in ['.md', '.mdx']:
        return 'markdown'
    elif file_ext in ['.html', '.htm']:
        return 'html'
    elif file_ext == '.txt':
        return 'text'
    elif file_ext == '.pdf':
        return 'pdf'
    else:
        raise ValueError(f"Unsupported file type: {file_ext}")


def validate_file_size(file_path: str, max_size_mb: int = 10) -> bool:
    """Validate if the file size is within the allowed limit."""
    file_size_mb = os.path.getsize(file_path) / (1024 * 1024)  # Convert bytes to MB
    return file_size_mb <= max_size_mb


def get_file_info(file_path: str) -> dict:
    """Get information about the file."""
    return {
        "name": Path(file_path).name,
        "size": os.path.getsize(file_path),
        "extension": Path(file_path).suffix,
        "type": get_file_type(file_path)
    }