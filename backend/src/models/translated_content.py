import uuid
import hashlib
from sqlalchemy import Column, String, Text, DateTime, Index, func
from sqlalchemy.dialects.postgresql import UUID

from .base import Base


class TranslatedContent(Base):
    """
    Stores LLM-translated chapter content, cached by profile hash.
    Translations are shared across users with the same profile preferences.
    
    The profile_hash is computed from: operating_system, programming_languages,
    preferred_explanation_style, prior_knowledge, learning_goals, industry
    
    This allows users with identical preferences to share the same translated content.
    """
    __tablename__ = "translated_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(String(100), nullable=False)
    source_language = Column(String(10), default="en", nullable=False)
    target_language = Column(String(10), default="ur", nullable=False)  # Urdu
    profile_hash = Column(String(64), nullable=False)  # SHA-256 hash of user profile
    original_content_hash = Column(String(64), nullable=False)  # SHA-256 hash of source content
    translated_content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=func.now(), nullable=False)
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)

    # Indexes for efficient cache lookup
    __table_args__ = (
        Index('idx_translated_chapter_profile_lang', 'chapter_id', 'profile_hash', 'target_language'),
        Index('idx_translated_content_hash', 'original_content_hash'),
    )

    def __repr__(self):
        return f"<TranslatedContent(id={self.id}, chapter='{self.chapter_id}', lang='{self.target_language}')>"

    @staticmethod
    def compute_profile_hash(
        operating_system: str | None,
        programming_languages: list[str] | None,
        preferred_explanation_style: str | None,
        prior_knowledge: list[str] | None,
        learning_goals: list[str] | None,
        industry: str | None
    ) -> str:
        """
        Compute a consistent hash from user profile preferences.
        Users with identical preferences will have the same hash,
        allowing them to share cached translations.
        """
        # Normalize and sort lists for consistent hashing
        profile_data = {
            "os": (operating_system or "").lower(),
            "langs": sorted([lang.lower() for lang in (programming_languages or [])]),
            "style": (preferred_explanation_style or "").lower(),
            "prior": sorted([p.lower() for p in (prior_knowledge or [])]),
            "goals": sorted([g.lower() for g in (learning_goals or [])]),
            "industry": (industry or "").lower()
        }
        
        # Create a deterministic string representation
        profile_str = str(profile_data)
        
        return hashlib.sha256(profile_str.encode()).hexdigest()
