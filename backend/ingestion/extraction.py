import requests
from bs4 import BeautifulSoup
from typing import Optional
import os
import sys

# From backend/ingestion/extraction.py, relative imports assume 'backend' is the top-level package

from config.logging_config import setup_logging # Corrected relative import

logger = setup_logging()

def extract_content(url: str) -> Optional[BeautifulSoup]:
    """
    Fetches a URL and extracts the main content area as a BeautifulSoup object.

    Args:
        url: The URL of the page to process.

    Returns:
        A BeautifulSoup object of the main content area, or None if an error occurs.
    """
    try:
        response = requests.get(url)
        response.raise_for_status()
    except requests.exceptions.RequestException as e:
        logger.error(f"Error fetching page {url}: {e}")
        return None

    soup = BeautifulSoup(response.content, 'html.parser')

    # Docusaurus typically wraps its main content in an <article> tag.
    # This is a common convention and a good starting point.
    article = soup.find('article')
    
    if not article:
        # As a fallback, try to find the main tag
        article = soup.find('main')

    if article:
        logger.info(f"Successfully extracted main content from {url}")
        return article
    else:
        logger.warning(f"Could not find <article> or <main> tag in {url}")
        return None

if __name__ == '__main__':
    # This block is for testing the script directly
    REAL_DOCS_URL = "https://docusaurus.io/docs"

    logger.info(f"Attempting to extract content from: {REAL_DOCS_URL}")
    
    content_soup = extract_content(REAL_DOCS_URL)
    
    if content_soup:
        logger.info(f"Successfully extracted content (first 500 chars of text): {content_soup.get_text(separator=' ', strip=True)[:500]}...")
    else:
        logger.error("Failed to extract content.")
