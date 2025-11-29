"""
Chatbot module for the Physical AI & Humanoid Robotics Textbook.

This module contains the RAG agent implementation using OpenAI Agents SDK with Gemini.
"""

from .agent import query_agent, rag_agent, search_book_content, qdrant_client

__all__ = [
    "query_agent",
    "rag_agent",
    "search_book_content",
    "qdrant_client",
]

