import requests
import xml.etree.ElementTree as ET
from typing import List
import os
import sys

# From backend/ingestion/discovery.py, relative imports assume 'backend' is the top-level package

from config.logging_config import setup_logging # Corrected relative import

logger = setup_logging()

def discover_urls(sitemap_url: str) -> List[str]:
    """
    Fetches a sitemap.xml file and parses it to extract all URLs.

    Args:
        sitemap_url: The URL of the sitemap.xml file.

    Returns:
        A list of URLs found in the sitemap.
    """
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()  # Raise an exception for bad status codes
    except requests.exceptions.RequestException as e:
        logger.error(f"Error fetching sitemap from {sitemap_url}: {e}")
        return []

    try:
        root = ET.fromstring(response.content)
        # XML namespace is often present, so we need to handle it
        namespace = '{http://www.sitemaps.org/schemas/sitemap/0.9}'
        urls = [
            elem.text for elem in root.findall(f".//{namespace}loc")
        ]
        logger.info(f"Successfully discovered {len(urls)} URLs from {sitemap_url}")
        return urls
    except ET.ParseError as e:
        logger.error(f"Error parsing XML from {sitemap_url}: {e}")
        return []

if __name__ == '__main__':
    # This block is for testing the script directly
    SITEMAP_URL = "https://physical-ai-humanoid-robotics-textb-beta-two.vercel.app/sitemap.xml"
    logger.info(f"Discovering URLs from {SITEMAP_URL}...")
    discovered_urls = discover_urls(SITEMAP_URL)
    
    if discovered_urls:
        logger.info(f"Found {len(discovered_urls)} URLs:")
        for url in discovered_urls:
            logger.info(url)
    else:
        logger.warning("No URLs found or an error occurred during discovery.")
