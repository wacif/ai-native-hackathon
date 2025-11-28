from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Integer, ARRAY
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from ..config.db import Base

class BookContent(Base):
    __tablename__ = "book_contents"

    content_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String, index=True, nullable=False)
    author = Column(String, nullable=False)
    chapter_number = Column(Integer, index=True, nullable=False)
    content_text = Column(Text, nullable=False)
    embedding = Column(ARRAY(Integer), nullable=True) # Store embeddings as an array of floats (or appropriate type)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

    # Foreign key for personalized content if applicable
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=True)
    user = relationship("User", backref="personalized_content")
