from bs4 import BeautifulSoup, Tag
from typing import List, Dict, Optional
import os
import sys

# Add the backend directory to the Python path for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from config.logging_config import setup_logging

logger = setup_logging()

# Constants for chunking strategy
TARGET_CHUNK_SIZE = 512
CHUNK_OVERLAP = 50

def count_tokens(text: str) -> int:
    """
    Estimates the number of tokens in a text by word count.
    A more accurate token count would use the specific tokenizer for the embedding model.
    """
    return len(text.split())

def get_heading_text(tag: Tag) -> Optional[str]:
    """Extracts text from a heading tag."""
    if tag and tag.name in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
        return tag.get_text(separator=' ', strip=True)
    return None

def chunk_content(soup_content: BeautifulSoup, source_url: str) -> List[Dict]:
    """
    Semantically chunks content from a BeautifulSoup object based on headings.
    
    Args:
        soup_content: A BeautifulSoup object representing the main content of a page.
        source_url: The URL of the page, used for metadata.

    Returns:
        A list of dictionaries, each representing a text chunk with metadata.
    """
    chunks = []
    current_chunk_text = ""
    current_section = "Introduction" # Default section
    chunk_index = 0

    # Find all heading tags and paragraph tags, and iterate in order
    content_tags = soup_content.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'p', 'li', 'div'])

    for tag in content_tags:
        if tag.name in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
            # If a new heading is encountered, flush the current chunk if not empty
            if current_chunk_text:
                chunks.append({
                    "text": current_chunk_text,
                    "section": current_section,
                    "chunk_index": chunk_index,
                    "source_url": source_url
                })
                chunk_index += 1
                current_chunk_text = "" # Reset for new chunk
            current_section = get_heading_text(tag) or current_section
            
            # Add the heading itself as part of the next chunk
            current_chunk_text += (get_heading_text(tag) + " ") if get_heading_text(tag) else ""
        elif tag.name in ['p', 'li', 'div']: # Consider paragraphs, list items and generic divs as content
            segment_text = tag.get_text(separator=' ', strip=True)
            if not segment_text:
                continue

            # Check if adding this segment exceeds target chunk size
            if count_tokens(current_chunk_text + segment_text) > TARGET_CHUNK_SIZE:
                # Flush current chunk
                chunks.append({
                    "text": current_chunk_text,
                    "section": current_section,
                    "chunk_index": chunk_index,
                    "source_url": source_url
                })
                chunk_index += 1

                # Start new chunk with overlap if possible
                overlap_text = current_chunk_text[-CHUNK_OVERLAP:] if len(current_chunk_text) > CHUNK_OVERLAP else ""
                current_chunk_text = overlap_text + segment_text + " "
            else:
                current_chunk_text += segment_text + " "

    # Add any remaining text as a final chunk
    if current_chunk_text:
        chunks.append({
            "text": current_chunk_text,
            "section": current_section,
            "chunk_index": chunk_index,
            "source_url": source_url
        })
    logger.info(f"Generated {len(chunks)} chunks for {source_url}")
    return chunks


if __name__ == '__main__':
    # Test the chunking logic with a dummy HTML
    dummy_html = """
    <html>
    <body>
        <main>
            <h1>Chapter 1: Introduction</h1>
            <p>This is the first paragraph of the introduction. It talks about many interesting things.</p>
            <p>Here is another paragraph, continuing the introduction. It also has fascinating details.</p>
            <h2>Section 1.1: Sub-heading One</h2>
            <p>Content for sub-heading one. This part is important.</p>
            <ul>
                <li>Item one</li>
                <li>Item two is longer and has more words.</li>
            </ul>
            <h3>Sub-sub-heading</h3>
            <div>More detailed information about a specific sub-topic. This div contains some text.</div>
            <p>Concluding paragraph for the sub-sub-heading section.</p>
            <h2>Section 1.2: Sub-heading Two</h2>
            <p>Content for sub-heading two. Very different from the first one.</p>
        </main>
    </body>
    </html>
    """
    from bs4 import BeautifulSoup
    dummy_soup = BeautifulSoup(dummy_html, 'html.parser')
    
    # We need to simulate the output of extract_content, which returns an article/main tag
    main_content = dummy_soup.find('main')
    if main_content:
        test_chunks = chunk_content(main_content, "http://dummy-url.com/chapter1")
        for i, chunk in enumerate(test_chunks):
            logger.info(f"--- Chunk {i} ---")
            logger.info(f"Section: {chunk['section']}")
            logger.info(f"Index: {chunk['chunk_index']}")
            logger.info(f"Text (first 100 chars): {chunk['text'][:100]}...")
            logger.info(f"Token count: {count_tokens(chunk['text'])}")
    else:
        logger.error("Could not find main content in dummy HTML.")