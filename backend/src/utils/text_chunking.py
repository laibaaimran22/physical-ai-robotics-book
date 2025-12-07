import re
from typing import List, Tuple
from tokenizers import Tokenizer
from tokenizers.models import BPE
from tokenizers.trainers import BpeTrainer
from tokenizers.pre_tokenizers import Whitespace
import nltk
from nltk.tokenize import sent_tokenize


# Download required NLTK data
try:
    nltk.data.find('tokenizers/punkt')
except LookupError:
    nltk.download('punkt')


def count_tokens(text: str) -> int:
    """
    Count approximate number of tokens in text.
    This is a simple heuristic - for production use, consider using actual tokenizers.
    """
    # Simple approach: split on whitespace and punctuation
    words = re.findall(r'\b\w+\b', text)
    return len(words)


def chunk_text_by_tokens(text: str, max_tokens: int = 400, min_tokens: int = 100) -> List[str]:
    """
    Split text into chunks based on token count.

    Args:
        text: Input text to chunk
        max_tokens: Maximum tokens per chunk
        min_tokens: Minimum tokens per chunk (if possible)

    Returns:
        List of text chunks
    """
    # Split text into sentences first
    sentences = sent_tokenize(text)

    chunks = []
    current_chunk = ""
    current_token_count = 0

    for sentence in sentences:
        sentence_token_count = count_tokens(sentence)

        # If adding this sentence would exceed max tokens
        if current_token_count + sentence_token_count > max_tokens:
            # If current chunk is substantial, save it
            if current_token_count >= min_tokens:
                chunks.append(current_chunk.strip())
                current_chunk = sentence
                current_token_count = sentence_token_count
            else:
                # If current chunk is too small, add the sentence anyway to avoid tiny chunks
                if current_chunk:
                    current_chunk += " " + sentence
                    current_token_count += sentence_token_count
                else:
                    current_chunk = sentence
                    current_token_count = sentence_token_count
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
            current_token_count += sentence_token_count

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def chunk_text_by_paragraph(text: str, max_tokens: int = 400) -> List[str]:
    """
    Split text into chunks based on paragraphs.

    Args:
        text: Input text to chunk
        max_tokens: Maximum tokens per chunk

    Returns:
        List of text chunks
    """
    # Split by paragraphs (double newlines)
    paragraphs = text.split('\n\n')

    chunks = []
    current_chunk = ""
    current_token_count = 0

    for paragraph in paragraphs:
        paragraph_token_count = count_tokens(paragraph)

        # If adding this paragraph would exceed max tokens and we already have content
        if current_token_count + paragraph_token_count > max_tokens and current_chunk:
            chunks.append(current_chunk.strip())
            current_chunk = paragraph
            current_token_count = paragraph_token_count
        else:
            # Add paragraph to current chunk
            if current_chunk:
                current_chunk += "\n\n" + paragraph
            else:
                current_chunk = paragraph
            current_token_count += paragraph_token_count

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def chunk_text(text: str, max_tokens: int = 400, min_tokens: int = 100, strategy: str = "sentence") -> List[str]:
    """
    Split text into chunks based on the specified strategy.

    Args:
        text: Input text to chunk
        max_tokens: Maximum tokens per chunk
        min_tokens: Minimum tokens per chunk
        strategy: Chunking strategy ("sentence" or "paragraph")

    Returns:
        List of text chunks
    """
    if strategy == "sentence":
        return chunk_text_by_tokens(text, max_tokens, min_tokens)
    elif strategy == "paragraph":
        return chunk_text_by_paragraph(text, max_tokens)
    else:
        raise ValueError(f"Unknown chunking strategy: {strategy}")


def validate_chunks(chunks: List[str], min_tokens: int = 50) -> List[Tuple[int, str]]:
    """
    Validate chunks to ensure they meet minimum requirements.

    Args:
        chunks: List of text chunks
        min_tokens: Minimum token count for a valid chunk

    Returns:
        List of tuples with (chunk_index, validation_message)
    """
    issues = []
    for i, chunk in enumerate(chunks):
        token_count = count_tokens(chunk)
        if token_count < min_tokens:
            issues.append((i, f"Chunk {i} has only {token_count} tokens, less than minimum {min_tokens}"))

    return issues


def merge_small_chunks(chunks: List[str], max_tokens: int = 400) -> List[str]:
    """
    Merge small chunks together if they're below the minimum threshold.

    Args:
        chunks: List of text chunks
        max_tokens: Maximum tokens per chunk (to avoid exceeding limits when merging)

    Returns:
        List of merged text chunks
    """
    if not chunks:
        return chunks

    merged_chunks = [chunks[0]]
    current_idx = 0

    for i in range(1, len(chunks)):
        current_chunk = merged_chunks[current_idx]
        next_chunk = chunks[i]

        # If current chunk is too small (less than 100 tokens) and combining with next won't exceed max
        if count_tokens(current_chunk) < 100 and count_tokens(current_chunk) + count_tokens(next_chunk) <= max_tokens:
            merged_chunks[current_idx] = current_chunk + " " + next_chunk
        else:
            # Add next chunk as a new chunk
            merged_chunks.append(next_chunk)
            current_idx += 1

    return merged_chunks