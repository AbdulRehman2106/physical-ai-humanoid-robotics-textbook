#!/usr/bin/env python3
"""
Content ingestion script for Physical AI textbook.
Parses MDX files, creates chunks, generates embeddings, and uploads to Qdrant.
"""

import os
import re
import sys
from pathlib import Path
from typing import List, Dict, Any
import hashlib

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import get_settings
from app.services.embeddings import EmbeddingService
from app.db.qdrant import QdrantDB
from app.models.document import DocumentChunk


class MDXParser:
    """Parser for MDX files."""

    def __init__(self, base_url: str = "https://physical-ai-textbook.vercel.app"):
        self.base_url = base_url

    def parse_file(self, file_path: Path, chapter_number: int) -> List[DocumentChunk]:
        """Parse an MDX file into chunks."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract chapter title from first H1
        chapter_title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        chapter_title = chapter_title_match.group(1) if chapter_title_match else f"Chapter {chapter_number}"

        # Split by H2 headings
        sections = re.split(r'\n##\s+', content)
        chunks = []

        # Process first section (before first H2)
        if sections[0].strip():
            intro_content = sections[0].strip()
            # Remove the H1 title
            intro_content = re.sub(r'^#\s+.+\n', '', intro_content, count=1)
            if intro_content.strip():
                chunk = self._create_chunk(
                    chapter_number=chapter_number,
                    chapter_title=chapter_title,
                    section_title="Introduction",
                    content=intro_content,
                    file_path=file_path
                )
                chunks.append(chunk)

        # Process remaining sections
        for section in sections[1:]:
            # Extract section title (first line)
            lines = section.split('\n', 1)
            section_title = lines[0].strip()
            section_content = lines[1] if len(lines) > 1 else ""

            if section_content.strip():
                # Check for subsections (H3)
                subsections = re.split(r'\n###\s+', section_content)

                # First subsection (content before first H3)
                if subsections[0].strip():
                    chunk = self._create_chunk(
                        chapter_number=chapter_number,
                        chapter_title=chapter_title,
                        section_title=section_title,
                        content=subsections[0].strip(),
                        file_path=file_path
                    )
                    chunks.append(chunk)

                # Process H3 subsections
                for subsection in subsections[1:]:
                    lines = subsection.split('\n', 1)
                    subsection_title = lines[0].strip()
                    subsection_content = lines[1] if len(lines) > 1 else ""

                    if subsection_content.strip():
                        chunk = self._create_chunk(
                            chapter_number=chapter_number,
                            chapter_title=chapter_title,
                            section_title=f"{section_title} - {subsection_title}",
                            content=subsection_content.strip(),
                            file_path=file_path
                        )
                        chunks.append(chunk)

        return chunks

    def _create_chunk(
        self,
        chapter_number: int,
        chapter_title: str,
        section_title: str,
        content: str,
        file_path: Path
    ) -> DocumentChunk:
        """Create a document chunk."""
        # Generate chunk ID
        chunk_id = hashlib.md5(
            f"{chapter_number}-{section_title}-{content[:100]}".encode()
        ).hexdigest()

        # Determine content type
        content_type = "text"
        if "```" in content:
            content_type = "code"
        elif ":::" in content:
            content_type = "callout"
        elif "quiz" in content.lower() or "question" in content.lower():
            content_type = "quiz"

        # Generate URL
        chapter_slug = file_path.parent.name
        section_slug = section_title.lower().replace(' ', '-').replace('/', '-')
        url = f"{self.base_url}/docs/chapters/{chapter_slug}#{section_slug}"

        # Extract keywords (simple approach)
        keywords = self._extract_keywords(content)

        # Count words
        word_count = len(content.split())

        return DocumentChunk(
            chunk_id=chunk_id,
            chapter_number=chapter_number,
            chapter_title=chapter_title,
            section_title=section_title,
            content=content,
            content_type=content_type,
            url=url,
            keywords=keywords,
            word_count=word_count
        )

    def _extract_keywords(self, content: str) -> List[str]:
        """Extract keywords from content."""
        # Simple keyword extraction - look for capitalized terms and technical terms
        keywords = set()

        # Common technical terms
        technical_terms = [
            'ROS', 'ROS 2', 'Gazebo', 'Isaac Sim', 'URDF', 'Physical AI',
            'VLA', 'simulation', 'robot', 'sensor', 'actuator', 'AI',
            'machine learning', 'neural network', 'embodied intelligence'
        ]

        content_lower = content.lower()
        for term in technical_terms:
            if term.lower() in content_lower:
                keywords.add(term)

        return list(keywords)[:10]  # Limit to 10 keywords


def main():
    """Main ingestion function."""
    print("[START] Starting content ingestion...")

    # Initialize services
    settings = get_settings()
    parser = MDXParser()
    embeddings = EmbeddingService()
    qdrant = QdrantDB()

    # Create collection
    print("[SETUP] Creating Qdrant collection...")
    qdrant.create_collection(vector_size=1024)

    # Find all chapter files
    docs_dir = Path(__file__).parent.parent.parent / "docs" / "chapters"
    chapter_files = sorted(docs_dir.glob("*/index.mdx"))

    # Filter out template
    chapter_files = [f for f in chapter_files if "_template" not in str(f)]

    print(f"[INFO] Found {len(chapter_files)} chapters")

    all_chunks = []

    # Parse all files
    for i, file_path in enumerate(chapter_files, 1):
        print(f"[PARSE] Chapter {i}: {file_path.parent.name}")
        chunks = parser.parse_file(file_path, chapter_number=i)
        all_chunks.extend(chunks)
        print(f"  [OK] Created {len(chunks)} chunks")

    print(f"\n[INFO] Total chunks: {len(all_chunks)}")

    # Generate embeddings in batches
    print("\n[EMBED] Generating embeddings...")
    batch_size = 10  # Reduced batch size to avoid timeouts
    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i:i + batch_size]
        batch_num = i//batch_size + 1
        total_batches = (len(all_chunks) + batch_size - 1) // batch_size
        print(f"  [BATCH] Processing batch {batch_num}/{total_batches}")

        try:
            # Get embeddings
            texts = [chunk.content for chunk in batch]
            vectors = embeddings.embed_texts(texts)

            # Upload to Qdrant (use model_dump instead of dict for Pydantic v2)
            chunk_dicts = [chunk.model_dump() for chunk in batch]
            qdrant.upsert_chunks(chunk_dicts, vectors)

            print(f"  [OK] Uploaded {len(batch)} chunks")
        except Exception as e:
            print(f"  [ERROR] Batch {batch_num} failed: {e}")
            print(f"  [RETRY] Retrying with smaller chunks...")
            # Retry with even smaller batches
            for j in range(0, len(batch), 5):
                mini_batch = batch[j:j+5]
                mini_texts = [chunk.content for chunk in mini_batch]
                mini_vectors = embeddings.embed_texts(mini_texts)
                mini_dicts = [chunk.model_dump() for chunk in mini_batch]
                qdrant.upsert_chunks(mini_dicts, mini_vectors)
                print(f"    [OK] Uploaded {len(mini_batch)} chunks (retry)")

    print("\n[COMPLETE] Ingestion complete!")
    print(f"[STATS] Total chunks indexed: {len(all_chunks)}")

    # Show collection info
    info = qdrant.get_collection_info()
    print(f"[STATS] Collection status: {info.status}")
    print(f"[STATS] Points count: {info.points_count}")


if __name__ == "__main__":
    main()
