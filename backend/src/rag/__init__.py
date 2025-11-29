"""
RAG (Retrieval-Augmented Generation) module for the Physical AI & Humanoid Robotics Textbook.

This module handles data ingestion and vector search functionality.
"""

from .ingestion import ingest_data, chunk_text, collect_markdown_files

__all__ = [
    "ingest_data",
    "chunk_text",
    "collect_markdown_files",
]

