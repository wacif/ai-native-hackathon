import uuid
from sqlalchemy import Column, String, Text, DateTime, ForeignKey, UniqueConstraint, Index, func
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship

from .base import Base


class PersonalizedContent(Base):
    """
    Stores LLM-generated personalized chapter content for each user.
    Content is cached here to avoid repeated LLM calls.
    """
    __tablename__ = "personalized_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey('users.id', ondelete='CASCADE'), nullable=False)
    chapter_id = Column(String(100), nullable=False)
    original_content_hash = Column(String(64), nullable=True)  # SHA-256 hash to detect source changes
    personalized_content = Column(Text, nullable=False)
    created_at = Column(DateTime, default=func.now(), nullable=False)
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now(), nullable=False)

    # Relationships
    user = relationship("User", backref="personalized_contents")

    # Constraints
    __table_args__ = (
        UniqueConstraint('user_id', 'chapter_id', name='uq_user_chapter'),
        Index('idx_personalized_user_chapter', 'user_id', 'chapter_id'),
    )

    def __repr__(self):
        return f"<PersonalizedContent(id={self.id}, user_id={self.user_id}, chapter_id='{self.chapter_id}')>"
